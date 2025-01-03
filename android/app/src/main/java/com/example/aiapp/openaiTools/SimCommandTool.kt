package com.example.aiapp.openaiTools

import com.example.aiapp.apiHost
import com.example.aiapp.models.api.SimCommandResponse
import com.example.aiapp.models.openai.Tool
import com.example.aiapp.utility.HttpMethod
import com.example.aiapp.utility.Http
import com.example.aiapp.utility.toJsonObject
import kotlinx.serialization.Serializable
import timber.log.Timber

@Serializable
data class SimCommandRequest(
    val command: Int
)

class SimCommand {
    companion object {
        val tool: Tool = Tool(
            name = "simCommand",
            description = "Send a simulation command",
            parameters = mapOf(
                "type" to "object",
                "properties" to mapOf(
                    "command" to mapOf(
                        "type" to "number"
                    )
                ),
                "required" to listOf("command")
            ).toJsonObject()
        )

        // make a HTTP POST /simcommand to send a simulation command
        /**
         * 1 = spawn cube
         * 2 = stop robot
         * 3 = start robot
         */
        suspend fun action(request: SimCommandRequest): SimCommandResponse {
            val body = mapOf(
                "command" to request.command
            )
            val response = Http.request<SimCommandResponse>(HttpMethod.POST, "$apiHost/simcommand", body = body)
            Timber.i("response: $response")
            return response
        }
    }
}