import omni.ext
import omni.ui as ui
import threading
from flask import Flask, request, jsonify
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.isaac.ui.menu import MenuItemDescription
from omni.isaac.ui.element_wrappers import ScrollingWindow
import carb
import carb.events
import omni.kit.app
import omni.timeline
from werkzeug.serving import make_server

import gc
import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .constants import server_ip

EXTENSION_TITLE = "My Extension"

# In-memory storage
north_bin_count = 0
west_bin_count = 0

# Create the Flask app
def create_app():
    app = Flask(__name__)

    @app.route('/bins', methods=['GET'])
    def get_bins():
        global north_bin_count, west_bin_count
        return jsonify({
            "north_bin_count": north_bin_count,
            "west_bin_count": west_bin_count
        }), 200

    @app.route('/simcommand', methods=['POST'])
    def simcommand():
        data = request.json
        if not data:
            return jsonify({"error": "No data provided"}), 400
        command = data['command']
        if not isinstance(command, (int, float)):
            return jsonify({"error": "Invalid command format. Must be a number."}), 400
        command = float(command)

        message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
        sim_command_msg = carb.events.type_from_string("simCommand")
        message_bus.push(sim_command_msg, payload={ "command": command })

        return jsonify({"message": "Success"}), 200

    return app

# ref: https://stackoverflow.com/a/45017691/16762230
class ServerThread(threading.Thread):
    def __init__(self, app):
        threading.Thread.__init__(self)
        self.server = make_server(server_ip, 5000, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        self.server.serve_forever()

    def shutdown(self):
        self.server.shutdown()

def start_server():
    global server
    app = create_app()
    # App routes defined here
    server = ServerThread(app)
    server.start()
    print('Flask server started')

def stop_server():
    global server
    server.shutdown()
    print('Flask server stopped')

# Extension class
class CompanyHelloWorldExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[company.hello.world] company hello world startup")

        self.ext_id = ext_id

        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=300, height=300, visible=True, dockPreference=ui.DockPreference.RIGHT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        with self._window.frame:
            with ui.VStack():
                self.label = ui.Label("Press Play to start the simulation.")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]
        add_menu_items(self._menu_items, EXTENSION_TITLE)

        self.server = None
        self.bin_trigger_sub = None

        # Events
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = None

        self._on_window(self._window.visible)

    def on_shutdown(self):
        print("[company.hello.world] company hello world shutdown")

        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        gc.collect()

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        print(f"Window visible: {visible}")
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
        else:
            self._usd_context = None
            self._stage_event_sub = None

    def on_bin_trigger(self, event: carb.events.IEvent):
        global north_bin_count, west_bin_count
        data = event.payload["bin"]
        print(f"on_bin_trigger = {data}")
        if data == "NorthBin":
            north_bin_count += 1
        elif data == "WestBin":
            west_bin_count += 1

    def _on_stage_event(self, event):
        # print(f"Stage event: {event.type}")
        global north_bin_count, west_bin_count
        if event.type == int(StageEventType.OMNIGRAPH_START_PLAY):
            # NOTE: this event gets triggered when starting and resuming simulation
            print(f"OMNIGRAPH_START_PLAY: {event.type}")
            # if self.server is None:
            start_server()
            self.label.text = f"Server running at http://{server_ip}:5000"
            # if self.bin_trigger_sub is None:
            message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
            bin_trigger_msg = carb.events.type_from_string("binTrigger")
            self.bin_trigger_sub = message_bus.create_subscription_to_pop_by_type(bin_trigger_msg, self.on_bin_trigger)
        elif event.type == int(StageEventType.OMNIGRAPH_STOP_PLAY):
            # NOTE: this event gets triggered when pausing and stopping simulation
            print(f"OMNIGRAPH_STOP_PLAY: {event.type}")
            # if self.server is not None:
            stop_server()
            self.label.text = "Press play button to start server."
            self.bin_trigger_sub = None

            north_bin_count = 0
            west_bin_count = 0
        # else:
        #     print(f"Unhandled event: {event.type}")