from isaacsim import SimulationApp

# Initialize the simulation app with specified settings
simulation_app = SimulationApp({
    "width": "1280",
    "height": "720",
    "headless": False
})

from omni.isaac.core.utils.extensions import enable_extension

import numpy as np
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.cortex.cortex_utils import load_behavior_module
from omni.isaac.cortex.cortex_world import Behavior, CortexWorld, LogicalStateMonitor
from omni.isaac.cortex.robot import add_franka_to_stage, CortexRobot, MotionCommandedRobot, CortexUr10, CortexGripper
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.cortex.dfb import DfDiagnosticsMonitor
from omni.isaac.cortex.cortex_utils import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.world import World
from decider_network import make_decider_network
import decider_network as decider_network
import omni
from omni.isaac.core.utils.stage import is_stage_loading
from typing import Dict, Optional, Sequence
import omni.isaac.motion_generation.interface_config_loader as icl
from omni.isaac.manipulators.grippers.gripper import Gripper
import carb
from pxr import AnimGraphSchema, Gf, Usd, UsdGeom, UsdSkel
from omni.isaac.core.articulations import Articulation, ArticulationSubset
import random
from omni.isaac.core.tasks import BaseTask
import omni.isaac.cortex.math_util as math_util
from omni.isaac.cortex.cortex_rigid_prim import CortexRigidPrim
from omni.isaac.cortex.cortex_utils import get_assets_root_path_or_die
from omni.isaac.core.objects import VisualCapsule, VisualSphere
from scipy.spatial.transform import Rotation
from omni.isaac.cortex.cortex_object import CortexObject
from omni.isaac.cortex.motion_commander import CortexObstacleType, MotionCommander
from omni.isaac.core.prims.xform_prim import XFormPrim
from typing import Optional, Sequence

import numpy as np
from omni.isaac.core.materials.physics_material import PhysicsMaterial
from omni.isaac.core.materials.preview_surface import PreviewSurface
from omni.isaac.core.materials.visual_material import VisualMaterial
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.string import find_unique_string_name
from pxr import Gf, UsdGeom

import carb.events
import omni.kit.app

from omni.isaac.core.scenes.scene import Scene

# Define a custom gripper class for the robot
class MyGripper(CortexGripper):
    def __init__(self, articulation: Articulation):
        """
        Initialize the custom gripper.

        Args:
            articulation (Articulation): The Articulation object containing the finger joints that will be controlled by this gripper.
        """
        super().__init__(
            articulation_subset=ArticulationSubset(articulation, ["finger_joint"]),
            opened_width=0,
            closed_width=0.79,  # found from testing in Isaac robot assembler tool
        )

    def joints_to_width(self, joint_positions: Sequence[float]) -> float:
        """
        Convert joint positions to gripper width.

        Args:
            joint_positions (Sequence[float]): The values for joints ["finger_joint"].

        Returns:
            float: The width of the gripper corresponding to those joint positions.
        """
        return joint_positions[0]

    def width_to_joints(self, width: float) -> np.ndarray:
        """
        Convert gripper width to joint positions.

        Args:
            width (float): The width of the gripper.

        Returns:
            np.ndarray: The values for joints ["finger_joint"] giving the requested gripper width.
        """
        return np.array([width])

# Define a custom UR10 robot class
class MyCortexUr10(MotionCommandedRobot):
    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ):
        """
        Initialize the custom UR10 robot.

        Args:
            name (str): A name for the UR10 robot.
            prim_path (str): The path to the UR10 prim in the USD stage.
            position (Optional[Sequence[float]]): The position of the robot.
            orientation (Optional[Sequence[float]]): The orientation of the robot.
        """
        motion_policy_config = icl.load_supported_motion_policy_config("UR10", "RMPflowCortex")
        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(smoothed_rmpflow=False, smoothed_commands=False),
        )
        # add a gripper commander to the robot
        self.gripper_commander = MyGripper(self)
        self.add_commander("gripper", self.gripper_commander)

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """
        Initialize the robot with the physics simulation view.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): The physics simulation view.
        """
        super().initialize(physics_sim_view=physics_sim_view)

# Utility functions to get transformations and positions of prims
def get_prim_transform(prim):
    """
    Get the transformation of a prim.

    Args:
        prim (Usd.Prim): The prim to get the transformation for.

    Returns:
        tuple: The translation and rotation of the prim.
    """
    gf_transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    gf_transform.Orthonormalize()
    gf_translation = gf_transform.ExtractTranslation()
    gf_rot = gf_transform.ExtractRotation()
    return gf_translation, gf_rot

def get_prim_pos(stage, prim_path, prim=None):
    """
    Get the position of a prim.

    Args:
        stage (Usd.Stage): The USD stage.
        prim_path (str): The path to the prim.
        prim (Usd.Prim, optional): The prim object.

    Returns:
        carb.Float3: The position of the prim.
    """
    if prim is None:
        prim = stage.GetPrimAtPath(prim_path)
    if prim:
        gf_translation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
        return carb.Float3(gf_translation[0], gf_translation[1], gf_translation[2])
    else:
        return None

def get_prim_rot_quat(stage, prim_path, prim=None):
    """
    Get the rotation quaternion of a prim.

    Args:
        stage (Usd.Stage): The USD stage.
        prim_path (str): The path to the prim.
        prim (Usd.Prim, optional): The prim object.

    Returns:
        Gf.Quatf: The rotation quaternion of the prim.
    """
    if prim is None:
        prim = stage.GetPrimAtPath(prim_path)
    if prim:
        gf_rotation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractRotationQuat()
        return gf_rotation
    else:
        return None

def prims_by_parent_path(parent_prim_path: str) -> list[Usd.Prim]:
    """
    Get prims by parent path.

    Args:
        parent_prim_path (str): The path to the parent prim.

    Returns:
        list[Usd.Prim]: The list of child prims.
    """
    stage: Usd.Stage = omni.usd.get_context().get_stage()
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    if not parent_prim.IsValid():
        return []
    return parent_prim.GetChildren()

# Function to generate a random spawn transform for bins
def random_bin_spawn_transform():
    """
    Generate a random spawn transform for bins.

    Returns:
        tuple: The position and quaternion of the spawn transform.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath("/World/Points/SpawnPoint")
    pos = get_prim_pos(stage, "/World/Points/SpawnPoint", prim)
    position = np.array([pos[0], pos[1], pos[2]])
    position[0] += random.random() * 0.2 - 0.1
    z = random.random() * 0.02 - 0.01
    w = random.random() * 0.02 - 0.01
    norm = np.sqrt(z**2 + w**2)
    quat = math_util.Quaternion([w / norm, 0, 0, z / norm])
    return position, quat.vals

# Global variables for scene and cubes
scene: Scene = None
cubes = []
last_spawned_cube = None

# Function to spawn a new cube
def spawn_cube():
    """
    Spawn a new cube in the scene.
    """
    global scene, cubes, last_spawned_cube
    if scene is None:
        return
    name = "bin_{}".format(len(cubes))
    prim_path = "/World/bins/{}".format(name)
    width = 0.09
    DynamicCuboid(
        prim_path=prim_path,
        name=name,
        size=width,
        color=np.array([0.7, 0.0, 0.0])
    )
    last_spawned_cube = scene.add(CortexRigidPrim(name=name, prim_path=prim_path))
    cubes.append(last_spawned_cube)
    x, q = random_bin_spawn_transform()
    last_spawned_cube.set_world_pose(position=x, orientation=q)
    last_spawned_cube.set_visibility(True)

# Define a task for bin stacking
class SpawnCubeTask(BaseTask):
    def __init__(self):
        """
        Initialize the bin stacking task.
        """
        super().__init__("bin_stacking")

    def post_reset(self) -> None:
        """
        Reset the task by clearing all spawned cubes.
        """
        global cubes, last_spawned_cube
        if len(cubes) > 0:
            for rigid_bin in cubes:
                self.scene.remove_object(rigid_bin.name)
            cubes.clear()
        last_spawned_cube = None

    def pre_step(self, time_step_index, simulation_time) -> None:
        """
        Spawn a new randomly oriented bin if the previous bin has been placed.

        Args:
            time_step_index (int): The current time step index.
            simulation_time (float): The current simulation time.
        """
        global last_spawned_cube
        spawn_new = False
        if last_spawned_cube is None:
            spawn_new = True
        else:
            (x, y, z), _ = last_spawned_cube.get_world_pose()
            is_on_conveyor = 0.3 < y < 4.5 and 0.2 < x < 0.7
            if not is_on_conveyor:
                spawn_new = True
        if spawn_new:
            spawn_cube()

# Event handler for simulation commands
def on_sim_command(event: carb.events.IEvent):
    """
    Handle simulation commands.
    1. Spawn a new cube.
    2. Stop the robot.
    3. Start the robot.

    Args:
        event (carb.events.IEvent): The event containing the simulation command.
    """
    data = event.payload["command"]
    print(f"on_sim_command = {data}")
    if data == 1:
        spawn_cube()
        print("spawned cube")
    elif data == 2:
        decider_network.can_run_robot = False
        print(f"can_run_robot = {decider_network.can_run_robot}")
    elif data == 3:
        decider_network.can_run_robot = True
        print(f"can_run_robot = {decider_network.can_run_robot}")

# Main function to run the simulation
def main():
    """
    Main function to run the simulation.
    """
    # enable custom extension to start Flask server
    enable_extension("company.hello.world")
    # enable Foxglove extension
    enable_extension("foxglove.tools.ws_bridge")
    simulation_app.update()
    # open stage
    omni.usd.get_context().open_stage("Scene.usd")
    # wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()
    print("Loading stage...")
    while is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    world: CortexWorld = CortexWorld()
    global scene
    scene = world.scene

    robot_position_path = "/World/Robot"
    robot_articulation_path = "/World/Robot/Robotiq_2F_140"
    stage_obj: Usd.Stage = omni.usd.get_context().get_stage()

    # print some information about the robot
    robot_prim = stage_obj.GetPrimAtPath(robot_position_path)
    position = get_prim_pos(stage_obj, robot_position_path, robot_prim)
    position = np.array([position[0], position[1], position[2]])
    print("position:", position)
    orientation = get_prim_rot_quat(stage_obj, robot_position_path, robot_prim)
    print("orientation:", orientation)
    pos, rot = get_prim_transform(robot_prim)
    print("pos:", pos)
    print("rot:", rot)
    rotation = Rotation.from_quat([orientation.GetReal(), orientation.GetImaginary()[0], orientation.GetImaginary()[1], orientation.GetImaginary()[2]])
    rotation_matrix = rotation.as_matrix()
    print("rotation_matrix:", rotation_matrix)

    robot: MotionCommandedRobot = world.add_robot(MyCortexUr10(
        name="robot",
        prim_path=robot_articulation_path
    ))

    # register all virtual obstacles under /World/Obstacles
    obstacles: list[Usd.Prim] = prims_by_parent_path("/World/Obstacles")
    for obstacle in obstacles:
        print(f"path: {obstacle.GetPath()}")
        stand = VisualCuboid(
            prim_path=str(obstacle.GetPath()),
            name=obstacle.GetName(),
        )
        robot.register_obstacle(stand)

    world.add_task(SpawnCubeTask())

    # get positions of drop points
    dropPointA = stage_obj.GetPrimAtPath("/World/Points/DropPointA")
    dropPointB = stage_obj.GetPrimAtPath("/World/Points/DropPointB")
    dropPointAPos = get_prim_pos(stage_obj, "/World/Points/DropPointA", dropPointA)
    dropPointAPos = np.array([dropPointAPos[0], dropPointAPos[1], dropPointAPos[2]])
    dropPointBPos = get_prim_pos(stage_obj, "/World/Points/DropPointB", dropPointB)
    dropPointBPos = np.array([dropPointBPos[0], dropPointBPos[1], dropPointBPos[2]])

    # add Cortex decider network
    world.add_decider_network(make_decider_network(robot, dropPointAPos, dropPointBPos))
    world.reset()

    # subscribe to message bus to listen for sim commands
    message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
    sim_command_msg = carb.events.type_from_string("simCommand")
    sim_command_sub = message_bus.create_subscription_to_pop_by_type(sim_command_msg, on_sim_command)

    # run the simulation
    world.run(simulation_app)
    simulation_app.close()

if __name__ == "__main__":
    main()
