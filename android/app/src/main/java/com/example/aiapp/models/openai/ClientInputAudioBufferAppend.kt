package com.example.aiapp.models.openai

import kotlinx.serialization.Serializable

@Serializable
data class ClientInputAudioBufferAppend(
    private val type: String = "input_audio_buffer.append",
    val audio: String // base64-encoded audio data
)