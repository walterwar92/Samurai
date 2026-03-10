package com.samurai.robotcontrol.voice

import android.content.Context
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.withContext
import okhttp3.OkHttpClient
import okhttp3.Request
import org.vosk.Model
import org.vosk.Recognizer
import org.vosk.android.RecognitionListener
import org.vosk.android.SpeechService
import org.json.JSONObject
import java.io.File
import java.io.FileOutputStream
import java.util.concurrent.TimeUnit
import java.util.zip.ZipInputStream

/**
 * Offline Russian speech recognition using Vosk.
 * Model is downloaded from the Vosk CDN on first launch and cached in filesDir.
 */
class VoskRecognizer(private val context: Context) {

    companion object {
        private const val TAG = "VoskRecognizer"
        private const val SAMPLE_RATE = 16000.0f
        private const val MODEL_NAME = "vosk-model-small-ru-0.22"
        private const val MODEL_URL =
            "https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip"
    }

    private var model: Model? = null
    private var speechService: SpeechService? = null

    private val _recognisedText = MutableStateFlow("")
    val recognisedText: StateFlow<String> = _recognisedText

    private val _isListening = MutableStateFlow(false)
    val isListening: StateFlow<Boolean> = _isListening

    private val _modelReady = MutableStateFlow(false)
    val modelReady: StateFlow<Boolean> = _modelReady

    private val _modelError = MutableStateFlow<String?>(null)
    val modelError: StateFlow<String?> = _modelError

    // -1 = not downloading; 0..100 = download progress %
    private val _downloadProgress = MutableStateFlow(-1)
    val downloadProgress: StateFlow<Int> = _downloadProgress

    // ── Initialise model (call once) ────────────────────────
    suspend fun initModel() = withContext(Dispatchers.IO) {
        try {
            val modelDir = File(context.filesDir, MODEL_NAME)
            if (!modelDir.exists()) {
                downloadAndExtract(modelDir)
            }
            model = Model(modelDir.absolutePath)
            _modelReady.value = true
            Log.i(TAG, "Vosk model ready at ${modelDir.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "initModel error", e)
            _modelError.value = e.message ?: "Ошибка загрузки модели"
        }
    }

    private fun downloadAndExtract(modelDir: File) {
        val zipFile = File(context.filesDir, "$MODEL_NAME.zip")
        try {
            Log.i(TAG, "Downloading model from $MODEL_URL")
            _downloadProgress.value = 0

            val client = OkHttpClient.Builder()
                .connectTimeout(30, TimeUnit.SECONDS)
                .readTimeout(120, TimeUnit.SECONDS)
                .build()

            val request = Request.Builder().url(MODEL_URL).build()
            client.newCall(request).execute().use { response ->
                if (!response.isSuccessful) throw Exception("HTTP ${response.code}")
                val body = response.body ?: throw Exception("Пустой ответ сервера")
                val contentLength = body.contentLength()
                var bytesRead = 0L

                FileOutputStream(zipFile).use { out ->
                    body.byteStream().use { input ->
                        val buffer = ByteArray(8192)
                        var n: Int
                        while (input.read(buffer).also { n = it } != -1) {
                            out.write(buffer, 0, n)
                            bytesRead += n
                            if (contentLength > 0) {
                                _downloadProgress.value =
                                    (bytesRead * 100 / contentLength).toInt()
                            }
                        }
                    }
                }
            }

            Log.i(TAG, "Extracting model zip")
            ZipInputStream(zipFile.inputStream().buffered()).use { zis ->
                var entry = zis.nextEntry
                while (entry != null) {
                    val outFile = File(context.filesDir, entry.name)
                    if (entry.isDirectory) {
                        outFile.mkdirs()
                    } else {
                        outFile.parentFile?.mkdirs()
                        FileOutputStream(outFile).use { zis.copyTo(it) }
                    }
                    zis.closeEntry()
                    entry = zis.nextEntry
                }
            }
            Log.i(TAG, "Model extracted to ${context.filesDir}/$MODEL_NAME")
        } finally {
            zipFile.delete()
            _downloadProgress.value = -1
        }
    }

    // ── Start listening ─────────────────────────────────────
    fun startListening() {
        val mdl = model ?: run { Log.w(TAG, "Model not ready"); return }
        if (_isListening.value) return

        val recognizer = Recognizer(mdl, SAMPLE_RATE)
        speechService = SpeechService(recognizer, SAMPLE_RATE).apply {
            startListening(object : RecognitionListener {
                override fun onPartialResult(hypothesis: String?) {}

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

                override fun onFinalResult(hypothesis: String?) = onResult(hypothesis)

                override fun onError(exception: Exception?) {
                    Log.e(TAG, "Recognition error", exception)
                }

                override fun onTimeout() { Log.i(TAG, "Timeout") }
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
