package com.example.aiapp.models

import kotlinx.serialization.Serializable

@Serializable
data class GetBinsResponse(
    val north_bin_count: Int,
    val west_bin_count: Int,
)