package com.example.aiapp.utility

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.json.Json
import okhttp3.Headers.Companion.toHeaders
import okhttp3.MediaType.Companion.toMediaTypeOrNull
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONObject
import timber.log.Timber
import java.net.URLEncoder
import java.nio.charset.StandardCharsets
import java.util.concurrent.TimeUnit

enum class HttpMethod {
    GET,
    POST,
    PUT,
    DELETE
}

class Http {
    companion object {
        suspend inline fun <reified T> request(
            method: HttpMethod,
            url: String,
            headers: Map<String, String> = emptyMap(),
            body: Map<String, Any>? = null
        ): T = withContext(Dispatchers.IO) {
            // url encode url
//            val url = URLEncoder.encode(url, StandardCharsets.UTF_8.toString())
            val tag = "$method $url"
            Timber.tag(tag).i("Sending $method request to: $url")

            // URL Request
            var request: Request.Builder = Request.Builder()
                .url(url)
            when (method) {
                HttpMethod.GET -> {
                    request = request.get()
                }

                HttpMethod.POST -> {
                    if (body == null) {
                        throw Exception("Body is null.")
                    }
                    val requestBody = JSONObject(body).toString()
                        .toRequestBody("application/json".toMediaTypeOrNull())
                    request = request.post(requestBody)
                }

                HttpMethod.PUT -> {
                    if (body == null) {
                        throw Exception("Body is null.")
                    }
                    val requestBody = JSONObject(body).toString()
                        .toRequestBody("application/json".toMediaTypeOrNull())
                    request = request.put(requestBody)
                }

                HttpMethod.DELETE -> {
                    request = request.delete()
                }
            }

            // headers & body
            val headers = headers.toMutableMap()
            if (method == HttpMethod.POST || method == HttpMethod.PUT) {
                headers["Content-Type"] = "application/json"
                Timber.tag(tag).i("body: $body")
            }
            request.headers(headers.toHeaders())

            // Create client and send request
            try {
                val client: OkHttpClient = OkHttpClient.Builder()
                    .connectTimeout(25, TimeUnit.SECONDS)
                    .writeTimeout(25, TimeUnit.SECONDS)
                    .readTimeout(25, TimeUnit.SECONDS)
                    .build()
                val response = client.newCall(request.build()).execute()

                // get the status code and headers
                val statusCode = response.code
                Timber.tag(tag).i("Status code: $statusCode")
                Timber.tag(tag).i("Headers: ${response.headers}")

                // get the raw string data
                val rawResponse = response.body?.string()
                Timber.tag(tag).i("Response: $rawResponse")

                if (!response.isSuccessful) {
                    throw Exception("An error occurred with response code: ${response.code}.")
                } else if (rawResponse.isNullOrEmpty()) {
                    throw Exception("No data was returned.")
                } else {
                    val jsonConfig = Json {
                        encodeDefaults = true
                        ignoreUnknownKeys = true
                        explicitNulls = false
                    }
                    val data = jsonConfig.decodeFromString<T>(rawResponse)
                    return@withContext data
                }
            } catch (e: Exception) {
                Timber.tag(tag).e(e)
                throw e
            }
        }
    }
}