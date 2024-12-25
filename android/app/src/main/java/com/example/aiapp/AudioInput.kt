package com.example.aiapp

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import android.media.audiofx.AcousticEchoCanceler
import android.media.audiofx.NoiseSuppressor
import java.util.Base64

/**
 * Class for handling audio input with echo cancellation and noise suppression.
 */
@SuppressLint("MissingPermission")
class AudioInput() {
    private val sampleRate = 24000
    private val minBufferSize = AudioRecord.getMinBufferSize(
        sampleRate,
        AudioFormat.CHANNEL_IN_MONO,
        AudioFormat.ENCODING_PCM_16BIT
    )
    @Volatile
    private var audioRecord: AudioRecord? = null
    private val audioLock = Any() // Lock object for thread-safety

    /**
     * Starts recording audio with echo cancellation and noise suppression.
     */
    fun startRecording() {
        synchronized(audioLock) {
            // check if audioRecord is already recording
            if (audioRecord?.recordingState == AudioRecord.RECORDSTATE_RECORDING) {
                return
            }

            audioRecord = AudioRecord(
                MediaRecorder.AudioSource.VOICE_COMMUNICATION,  // if you go to definition, it says it optimizes for echo cancellation
                sampleRate,
                AudioFormat.CHANNEL_IN_MONO,
                AudioFormat.ENCODING_PCM_16BIT,
                minBufferSize
            )

            audioRecord?.audioSessionId?.let { audioSessionId ->
                if (AcousticEchoCanceler.isAvailable()) {
                    val echoCanceler = AcousticEchoCanceler.create(audioSessionId)
                    echoCanceler?.setEnabled(true)
                }

                if (NoiseSuppressor.isAvailable()) {
                    val noiseSuppressor = NoiseSuppressor.create(audioSessionId)
                    noiseSuppressor?.setEnabled(true)
                }
            }

            audioRecord?.startRecording()
        }
    }

    /**
     * Reads audio data from the microphone and returns it as a Base64 encoded string.
     * @return Base64 encoded audio data, or null if not recording.
     */
    fun read(): String? {
        synchronized(audioLock) {
            val currentAudioRecord = audioRecord ?: return null

            // Check that audioRecord is still recording
            if (currentAudioRecord.recordingState != AudioRecord.RECORDSTATE_RECORDING) {
                return null
            }

            val buffer = ByteArray(minBufferSize)
            val size = buffer.size
            val bytesRead = currentAudioRecord.read(buffer, 0, size)

            val base64Data = Base64.getEncoder().encodeToString(buffer.copyOfRange(0, bytesRead))

            return base64Data
        }
    }

    /**
     * Stops recording and releases the audio resources.
     */
    fun close() {
        synchronized(audioLock) {
            audioRecord?.let {
                if (it.recordingState == AudioRecord.RECORDSTATE_RECORDING) {
                    it.stop()
                }
                it.release()
            }
            audioRecord = null // Set to null to prevent future access
        }
    }
}