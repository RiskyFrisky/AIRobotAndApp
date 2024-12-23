package com.example.aiapp.openaiTools

import com.example.aiapp.apiHost
import com.example.aiapp.models.GetBinsResponse
import com.example.aiapp.models.SimCommandResponse
import com.example.aiapp.models.Tool
import com.example.aiapp.utility.Http
import com.example.aiapp.utility.HttpMethod
import timber.log.Timber

class GetBins {
    companion object {
        val tool: Tool = Tool(
            name = "getBins",
            description = "Gets information about the bins",
        )

        suspend fun action(): GetBinsResponse {
            val response = Http.request<GetBinsResponse>(HttpMethod.GET, "$apiHost/bins")
            Timber.i("response: $response")
            return response
        }
    }
}