package com.example.aiapp.utility

import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive

@Suppress("UNCHECKED_CAST")
fun Map<String, Any?>.toJsonObject(): JsonObject {
    return JsonObject(this.mapValues { (_, value) ->
        when (value) {
            is String -> JsonPrimitive(value)
            is Number -> JsonPrimitive(value)
            is Boolean -> JsonPrimitive(value)
            is Map<*, *> -> (value as Map<String, Any?>).toJsonObject()  // Recursive call for nested maps
            is List<*> -> JsonArray(value.map { listItem ->
                when (listItem) {
                    is String -> JsonPrimitive(listItem)
                    is Number -> JsonPrimitive(listItem)
                    is Boolean -> JsonPrimitive(listItem)
                    is Map<*, *> -> (listItem as Map<String, Any?>).toJsonObject()
                    else -> JsonNull  // Handle null or unsupported types
                }
            })
            else -> JsonNull  // Handle null or unsupported types
        }
    })
}