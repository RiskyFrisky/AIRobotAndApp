package com.example.aiapp.models

import kotlinx.serialization.Serializable

@Serializable
data class ServerResponseTextDelta(
    val event_id: String, // Unique ID of the server event
    val type: String, // Must be "response.audio_transcript.delta"
    val response_id: String, // ID of the response
    val item_id: String, // ID of the item
    val output_index: Int, // Index of the output item in the response
    val content_index: Int, // Index of the content part in the item's content array
    val delta: String // Base64-encoded audio data delta.
)