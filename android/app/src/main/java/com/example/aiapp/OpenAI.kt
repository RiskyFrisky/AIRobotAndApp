package com.example.aiapp

import com.example.aiapp.models.ClientInputAudioBufferAppend
import com.example.aiapp.models.ClientSessionUpdate
import com.example.aiapp.models.ServerResponseTextDone
import com.microsoft.cognitiveservices.speech.StreamStatus
import io.ktor.client.HttpClient
import io.ktor.client.engine.okhttp.OkHttp
import io.ktor.client.plugins.defaultRequest
import io.ktor.client.plugins.websocket.DefaultClientWebSocketSession
import io.ktor.client.plugins.websocket.WebSockets
import io.ktor.client.plugins.websocket.webSocket
import io.ktor.client.request.header
import io.ktor.websocket.Frame
import io.ktor.websocket.readText
import io.ktor.websocket.send
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import timber.log.Timber

enum class OpenAIState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED
}

class OpenAI {
    private val backgroundCoroutineScope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private var websocketSession: DefaultClientWebSocketSession? = null
    private val jsonConfig = Json {
        encodeDefaults = true
        ignoreUnknownKeys = true
        explicitNulls = false
    }

    suspend fun startWebsocket(
        onSessionState: (OpenAIState) -> Unit,
        onUserTalking: () -> Unit,
        onBotResponse: (String) -> Unit
    ) {
        onSessionState(OpenAIState.CONNECTING)

        // Create an HTTP client with WebSocket support
        val client = HttpClient(OkHttp) {
            install(WebSockets)

            // Add the default request headers
            defaultRequest {
                header(
                    "Authorization",
                    "Bearer $openaiKey"
                )
                header("OpenAI-Beta", "realtime=v1")
            }
        }

        // Define the WebSocket URL
        val url = "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview"

        // Connect to the WebSocket server
        try {
            client.webSocket(url) {
                websocketSession = this
                Timber.i("Connected to server.")
                onSessionState(OpenAIState.CONNECTED)

                val sessionUpdate = ClientSessionUpdate()
                val jsonStringSessionUpdate = jsonConfig.encodeToString(
                    ClientSessionUpdate.serializer(),
                    sessionUpdate
                )
                send(jsonStringSessionUpdate)

                while (true) {
                    when (val message = incoming.receive()) {
                        is Frame.Text -> {
                            try {
                                // Read the incoming message as a JSON string
                                val jsonString = message.readText()
                                // Parse the JSON string into a JsonElement
                                val jsonElement: JsonElement = jsonConfig.parseToJsonElement(jsonString)
                                val type: String = jsonElement.jsonObject["type"]?.jsonPrimitive?.content ?: throw Exception("Missing 'type' field")
                                // Check if the received message is an error event
                                if (type == "error") {
                                    Timber.e("Error: $jsonElement")
                                } else if (type == "rate_limits.updated") {
                                    Timber.i("Rate limits updated: $jsonElement")
                                } else if (type == "input_audio_buffer.speech_started") {
                                    Timber.i("User started talking.")
                                    onUserTalking()
                                } else if (type == "response.text.done") {
                                    val event = jsonConfig.decodeFromJsonElement(
                                        ServerResponseTextDone.serializer(),
                                        jsonElement
                                    )
                                    Timber.i("Bot: ${event.text}")
                                    onBotResponse(event.text)
                                } else {
//                                    Timber.i("Unknown event: $jsonElement")
                                }
                            } catch (e: Exception) {
                                Timber.e(e)
                            }
                        }
                        is Frame.Close -> {
                            Timber.i("Connection closed.")
                            break
                        }
                        else -> Unit
                    }
                }
            }
        } catch (e: Exception) {
            Timber.e(e)
        }

        websocketSession = null
        // Close the client
        client.close()

        onSessionState(OpenAIState.DISCONNECTED)
    }

    fun sendInputAudioToWebsocket(base64Data: String) {
        backgroundCoroutineScope.launch {
            val event = ClientInputAudioBufferAppend(
                audio = base64Data
            )
            val jsonString = jsonConfig.encodeToString(ClientInputAudioBufferAppend.serializer(), event)
            websocketSession?.send(jsonString)
        }
    }
}