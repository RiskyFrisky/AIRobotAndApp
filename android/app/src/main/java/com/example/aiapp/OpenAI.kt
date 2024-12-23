package com.example.aiapp

import com.example.aiapp.models.ClientConversationItemCreateFunctionOutput
import com.example.aiapp.models.ClientInputAudioBufferAppend
import com.example.aiapp.models.ClientResponseCreate
import com.example.aiapp.models.ClientSessionUpdate
import com.example.aiapp.models.FunctionCallItem
import com.example.aiapp.models.ServerResponseFunctionCallArgumentsDone
import com.example.aiapp.models.ServerResponseTextDelta
import com.example.aiapp.models.ServerResponseTextDone
import com.example.aiapp.openaiTools.GetBins
import com.example.aiapp.openaiTools.SimCommand
import com.example.aiapp.openaiTools.SimCommandRequest
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
    private var client: HttpClient? = null
    private var websocketSession: DefaultClientWebSocketSession? = null
    private val jsonConfig = Json {
        encodeDefaults = true
        ignoreUnknownKeys = true
        explicitNulls = false
    }

    private var onSessionState: (OpenAIState) -> Unit = {}

    suspend fun startWebsocket(
        onSessionState: (OpenAIState) -> Unit,
        onUserTalking: () -> Unit,
        onBotDeltaResponse: (String) -> Unit,
        onBotResponse: (String) -> Unit
    ) {
        this.onSessionState = onSessionState

        stopWebsocket()

        onSessionState(OpenAIState.CONNECTING)

        // Create an HTTP client with WebSocket support
        client = HttpClient(OkHttp) {
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
            client?.webSocket(url) {
                websocketSession = this
                Timber.i("Connected to server.")
                onSessionState(OpenAIState.CONNECTED)

                val sessionUpdate = ClientSessionUpdate()
                val jsonStringSessionUpdate = jsonConfig.encodeToString(
                    ClientSessionUpdate.serializer(),
                    sessionUpdate
                )
                send(jsonStringSessionUpdate)

                while (client != null) {
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
                                } else if (type == "response.text.delta") {
                                    val event = jsonConfig.decodeFromJsonElement(
                                        ServerResponseTextDelta.serializer(),
                                        jsonElement
                                    )
//                                    Timber.i("Bot: ${event.delta}")
                                    onBotDeltaResponse(event.delta)
                                } else if (type == "response.text.done") {
                                    val event = jsonConfig.decodeFromJsonElement(
                                        ServerResponseTextDone.serializer(),
                                        jsonElement
                                    )
                                    Timber.i("Bot: ${event.text}")
                                    onBotResponse(event.text)
                                } else if (type == "response.function_call_arguments.done") {
                                    val event = jsonConfig.decodeFromJsonElement(
                                        ServerResponseFunctionCallArgumentsDone.serializer(),
                                        jsonElement
                                    )
                                    val functionName = event.name
                                    val arguments = event.arguments
                                    val callId = event.call_id
                                    Timber.i("Function call: $functionName, args: $arguments")

                                    backgroundCoroutineScope.launch {
                                        suspend fun setFunctionOutput(output: String) {
                                            val response =
                                                ClientConversationItemCreateFunctionOutput(
                                                    item = FunctionCallItem(
                                                        call_id = callId,
                                                        output = output
                                                    )
                                                )
                                            send(
                                                jsonConfig.encodeToString(
                                                    ClientConversationItemCreateFunctionOutput.serializer(),
                                                    response
                                                )
                                            )
                                        }

                                        try {
                                            when (functionName) {
                                                SimCommand.tool.name -> {
                                                    arguments ?: throw Exception("Missing arguments")
                                                    val request =
                                                        jsonConfig.decodeFromString(
                                                            SimCommandRequest.serializer(),
                                                            arguments
                                                        )
                                                    val response = SimCommand.action(request)
                                                    setFunctionOutput(response.toString())
                                                    createResponse()
                                                }
                                                GetBins.tool.name -> {
                                                    val response = GetBins.action()
                                                    setFunctionOutput(response.toString())
                                                    createResponse()
                                                }
                                                else -> throw Exception("Unknown function: $functionName")
                                            }
                                        } catch (e: Exception) {
                                            Timber.e(e, "Error for ${event.type}")
                                            try {
                                                setFunctionOutput("Error for function $functionName: ${e.localizedMessage}")
                                                createResponse()
                                            } catch (e: Exception) {
                                                Timber.e(e)
                                            }
                                        }
                                    }
                                } else {
                                    Timber.w("Unknown event: $jsonElement")
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

        stopWebsocket()
    }

    fun stopWebsocket() {
        websocketSession = null
        // Close the client
        client?.close()
        client = null
        onSessionState(OpenAIState.DISCONNECTED)

        Timber.i("Disconnected from server.")
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

    // MARK: - Utils
    private suspend fun createResponse() {
        // request a response
        val response = ClientResponseCreate()
        val jsonStringResponse = jsonConfig.encodeToString(
            ClientResponseCreate.serializer(),
            response
        )
        websocketSession?.send(jsonStringResponse)
    }
}