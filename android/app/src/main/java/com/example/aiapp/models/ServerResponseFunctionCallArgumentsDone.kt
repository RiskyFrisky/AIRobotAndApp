package com.example.aiapp.models

import kotlinx.serialization.Serializable

@Serializable
data class ServerResponseFunctionCallArgumentsDone(
    val event_id: String, // Unique ID of the server event
    val type: String, // Must be "response.function_call_arguments.done "
    val response_id: String, // ID of the response
    val item_id: String, // ID of the item
    val output_index: Int, // Index of the output item in the response
    val call_id: String,
    val name: String, // Name of the function
    val arguments: String?
)