package com.example.aiapp.models.openai

import kotlinx.serialization.Serializable

@Serializable
data class ClientConversationItemCreateFunctionOutput(
    private val type: String = "conversation.item.create",
    val item: FunctionCallItem
)

@Serializable
data class FunctionCallItem(
    private val type: String = "function_call_output",
    val call_id: String,
    val output: String,
)