package com.example.aiapp.models.openai

import kotlinx.serialization.Serializable

@Serializable
data class ServerResponseTextDone(
    val event_id: String, // Unique ID of the server event
    val type: String, // Must be "response.text.done"
    val response_id: String, // ID of the response
    val item_id: String, // ID of the item
    val output_index: Int, // Index of the output item in the response
    val content_index: Int, // Index of the content part in the item's content array
    val text: String // Base64-encoded text data delta.
)