package com.example.aiapp.models.api

import kotlinx.serialization.Serializable

@Serializable
data class SimCommandResponse(
    val message: String
)