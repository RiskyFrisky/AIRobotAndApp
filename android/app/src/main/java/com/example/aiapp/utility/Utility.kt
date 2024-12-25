package com.example.aiapp.utility

import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.serialization.json.Json
import timber.log.Timber

object Utility {
    val jsonConfig = Json {
        encodeDefaults = true
        ignoreUnknownKeys = true
        explicitNulls = false
    }

    val coroutineExceptionHandler = CoroutineExceptionHandler{ coroutineContext, throwable ->
        Timber.e(throwable, "Coroutine exception in ${coroutineContext[CoroutineExceptionHandler.Key]}")
    }
}