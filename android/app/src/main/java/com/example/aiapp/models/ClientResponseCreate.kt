package com.example.aiapp.models

import kotlinx.serialization.Serializable

@Serializable
data class ClientResponseCreate(
    private val type: String = "response.create", // Event type
)