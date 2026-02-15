package com.samurai.robotcontrol.voice

import android.content.Context
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.withContext
import org.vosk.Model
import org.vosk.Recognizer
import org.vosk.android.RecognitionListener
import org.vosk.android.SpeechService
import org.vosk.android.StorageService
import org.json.JSONObject

/**
 * Offline Russian speech recognition using Vosk.
 * Model: vosk-model-small-ru-0.22  (downloaded to app assets or external storage).
 *
 * Emits recognised text via [recognisedText] StateFlow.
 */
class VoskRecognizer(private val context: Context) {

    companion object {
        private const val TAG = "VoskRecognizer"
        private const val SAMPLE_RATE = 16000.0f
        private const val MODEL_NAME = "vosk-model-small-ru-0.22"
    }

    private var model: Model? = null
    private var speechService: SpeechService? = null

    private val _recognisedText = MutableStateFlow("")
    val recognisedText: StateFlow<String> = _recognisedText

    private val _isListening = MutableStateFlow(false)
    val isListening: StateFlow<Boolean> = _isListening

    private val _modelReady = MutableStateFlow(false)
    val modelReady: StateFlow<Boolean> = _modelReady

    // ── Initialise model (call once) ────────────────────────
    suspend fun initModel() = withContext(Dispatchers.IO) {
        try {
            StorageService.unpack(context, MODEL_NAME, "model",
                { mdl ->
                    model = mdl
                    _modelReady.value = true
                    Log.i(TAG, "Vosk model ready")
                },
                { exception ->
                    Log.e(TAG, "Model init failed", exception)
                }
            )
        } catch (e: Exception) {
            Log.e(TAG, "initModel error", e)
        }
    }

    // ── Start listening ─────────────────────────────────────
    fun startListening() {
        val mdl = model ?: run {
            Log.w(TAG, "Model not ready")
            return
        }
        if (_isListening.value) return

        val recognizer = Recognizer(mdl, SAMPLE_RATE)
        speechService = SpeechService(recognizer, SAMPLE_RATE).apply {
            startListening(object : RecognitionListener {
                override fun onPartialResult(hypothesis: String?) {
                    // optional: show partial text
                }

                override fun onResult(hypothesis: String?) {
                    if (hypothesis == null) return
                    try {
                        val text = JSONObject(hypothesis).optString("text", "").trim()
                        if (text.isNotEmpty()) {
                            Log.i(TAG, "Result: $text")
                            _recognisedText.value = text
                        }
                    } catch (e: Exception) {
                        Log.e(TAG, "Parse error", e)
                    }
                }

                override fun onFinalResult(hypothesis: String?) {
                    onResult(hypothesis)
                }

                override fun onError(exception: Exception?) {
                    Log.e(TAG, "Recognition error", exception)
                }

                override fun onTimeout() {
                    Log.i(TAG, "Timeout")
                }
            })
        }
        _isListening.value = true
        Log.i(TAG, "Listening started")
    }

    // ── Stop listening ──────────────────────────────────────
    fun stopListening() {
        speechService?.stop()
        speechService = null
        _isListening.value = false
        Log.i(TAG, "Listening stopped")
    }

    // ── Cleanup ─────────────────────────────────────────────
    fun destroy() {
        stopListening()
        model?.close()
        model = null
    }
}
