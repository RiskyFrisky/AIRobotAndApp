package com.example.aiapp.models

import kotlinx.serialization.Serializable
import kotlinx.serialization.json.JsonObject

@Serializable
data class ClientSessionUpdate(
    private val type: String = "session.update",
    private val session: Session = Session()
)

@Serializable
data class Session(
    val modalities: List<String> = listOf("text"),
    val instructions: String = "Please assist the user. Keep all responses less than 100 words.",
    val input_audio_format: String = "pcm16",
    val turn_detection: TurnDetection = TurnDetection(),
    val tools: List<Tool> = availableTools,
    val tool_choice: String = "auto",
    val temperature: Double = 0.8,
    val max_response_output_tokens: Int = 150
)

@Serializable
data class TurnDetection(
    val type: String = "server_vad",
    val threshold: Double = 0.5,
    val prefix_padding_ms: Int = 300,
    val silence_duration_ms: Int = 500
)

@Serializable
data class Tool(
    private val type: String = "function",
    val name: String,
    val description: String,
    val parameters: JsonObject? = null
)

val availableTools: List<Tool> = listOf(
)