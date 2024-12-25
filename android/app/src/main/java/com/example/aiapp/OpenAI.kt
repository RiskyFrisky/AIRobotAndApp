package com.example.aiapp

import com.example.aiapp.models.openai.ClientConversationItemCreateFunctionOutput
import com.example.aiapp.models.openai.ClientInputAudioBufferAppend
import com.example.aiapp.models.openai.ClientResponseCreate
import com.example.aiapp.models.openai.ClientSessionUpdate
import com.example.aiapp.models.openai.FunctionCallItem
import com.example.aiapp.models.openai.OpenAISessionState
import com.example.aiapp.models.openai.ServerResponseFunctionCallArgumentsDone
import com.example.aiapp.models.openai.ServerResponseTextDelta
import com.example.aiapp.models.openai.ServerResponseTextDone
import com.example.aiapp.openaiTools.GetBins
import com.example.aiapp.openaiTools.SimCommand
import com.example.aiapp.openaiTools.SimCommandRequest
import com.example.aiapp.utility.Utility
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
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import timber.log.Timber

/**
 * Class responsible for managing the WebSocket connection to the OpenAI API.
 */
class OpenAI {
    private val backgroundCoroutineScope = CoroutineScope(Dispatchers.IO + SupervisorJob() + Utility.coroutineExceptionHandler)

    private var client: HttpClient? = null
    private var websocketSession: DefaultClientWebSocketSession? = null

    private var onSessionState: (OpenAISessionState) -> Unit = {}

    /**
     * Starts the WebSocket connection to the OpenAI API.
     *
     * @param onSessionState Callback to handle changes in the WebSocket session state.
     * @param onUserTalking Callback to handle when the user starts talking.
     * @param onBotDeltaResponse Callback to handle partial responses from the bot.
     * @param onBotResponse Callback to handle complete responses from the bot.
     */
    suspend fun startWebsocket(
        onSessionState: (OpenAISessionState) -> Unit,
        onUserTalking: () -> Unit,
        onBotDeltaResponse: (String) -> Unit,
        onBotResponse: (String) -> Unit
    ) {
        this.onSessionState = onSessionState

        // Ensure any existing WebSocket connection is stopped
        stopWebsocket()

        // Update the session state to CONNECTING
        onSessionState(OpenAISessionState.CONNECTING)

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
                onSessionState(OpenAISessionState.CONNECTED)

                // Send a session update message
                val sessionUpdate = ClientSessionUpdate()
                val jsonStringSessionUpdate = Utility.jsonConfig.encodeToString(
                    ClientSessionUpdate.serializer(),
                    sessionUpdate
                )
                send(jsonStringSessionUpdate)

                // Listen for incoming messages
                while (client != null) {
                    when (val message = incoming.receive()) {
                        is Frame.Text -> {
                            try {
                                // Read the incoming message as a JSON string
                                val jsonString = message.readText()
                                // Parse the JSON string into a JsonElement
                                val jsonElement: JsonElement = Utility.jsonConfig.parseToJsonElement(jsonString)
                                val type: String = jsonElement.jsonObject["type"]?.jsonPrimitive?.content ?: throw Exception("Missing 'type' field")

                                // Handle different types of messages
                                when (type) {
                                    "error" -> {
                                        Timber.e("Error: $jsonElement")
                                    }
                                    "rate_limits.updated" -> {
                                        Timber.i("Rate limits updated: $jsonElement")
                                    }
                                    "input_audio_buffer.speech_started" -> {
                                        Timber.i("User started talking.")
                                        onUserTalking()
                                    }
                                    "response.text.delta" -> {
                                        val event = Utility.jsonConfig.decodeFromJsonElement(
                                            ServerResponseTextDelta.serializer(),
                                            jsonElement
                                        )
                                        onBotDeltaResponse(event.delta)
                                    }
                                    "response.text.done" -> {
                                        val event = Utility.jsonConfig.decodeFromJsonElement(
                                            ServerResponseTextDone.serializer(),
                                            jsonElement
                                        )
                                        Timber.i("Bot: ${event.text}")
                                        onBotResponse(event.text)
                                    }
                                    "response.function_call_arguments.done" -> {
                                        val event = Utility.jsonConfig.decodeFromJsonElement(
                                            ServerResponseFunctionCallArgumentsDone.serializer(),
                                            jsonElement
                                        )
                                        val functionName = event.name
                                        val arguments = event.arguments
                                        val callId = event.call_id
                                        Timber.i("Function call: $functionName, args: $arguments")

                                        // Handle function call in a background coroutine
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
                                                    Utility.jsonConfig.encodeToString(
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
                                                            Utility.jsonConfig.decodeFromString(
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
                                    }
                                    else -> {
                                        Timber.w("Unknown event: $jsonElement")
                                    }
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

        // Ensure the WebSocket connection is stopped
        stopWebsocket()
    }

    /**
     * Stops the WebSocket connection to the OpenAI API.
     */
    fun stopWebsocket() {
        websocketSession = null
        // Close the client
        client?.close()
        client = null
        onSessionState(OpenAISessionState.DISCONNECTED)

        Timber.i("Disconnected from server.")
    }

    /**
     * Sends audio input data to the WebSocket connection.
     *
     * @param base64Data The audio data encoded in Base64 format.
     */
    fun sendInputAudioToWebsocket(base64Data: String) {
        backgroundCoroutineScope.launch {
            val event = ClientInputAudioBufferAppend(
                audio = base64Data
            )
            val jsonString = Utility.jsonConfig.encodeToString(ClientInputAudioBufferAppend.serializer(), event)
            websocketSession?.send(jsonString)
        }
    }

    // MARK: - Utils

    /**
     * Requests a response from the OpenAI API.
     */
    private suspend fun createResponse() {
        // Request a response
        val response = ClientResponseCreate()
        val jsonStringResponse = Utility.jsonConfig.encodeToString(
            ClientResponseCreate.serializer(),
            response
        )
        websocketSession?.send(jsonStringResponse)
    }
}