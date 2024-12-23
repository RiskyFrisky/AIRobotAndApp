package com.example.aiapp

import android.Manifest
import android.annotation.SuppressLint
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.media.AudioManager
import android.net.Uri
import android.os.Bundle
import android.util.Base64
import android.webkit.WebView
import android.widget.Button
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.gestures.detectHorizontalDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.DrawerValue
import androidx.compose.material3.ModalDrawerSheet
import androidx.compose.material3.ModalNavigationDrawer
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.rememberDrawerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.app.ActivityCompat
import androidx.lifecycle.lifecycleScope
import com.example.aiapp.ui.theme.AIAppTheme
import com.microsoft.cognitiveservices.speech.Connection
import com.microsoft.cognitiveservices.speech.SpeechConfig
import com.microsoft.cognitiveservices.speech.SpeechSynthesisCancellationDetails
import com.microsoft.cognitiveservices.speech.SpeechSynthesisEventArgs
import com.microsoft.cognitiveservices.speech.SpeechSynthesizer
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.launch
import org.json.JSONException
import org.json.JSONObject
import org.webrtc.AudioTrack
import org.webrtc.DataChannel
import org.webrtc.DefaultVideoDecoderFactory
import org.webrtc.DefaultVideoEncoderFactory
import org.webrtc.EglBase
import org.webrtc.EglRenderer
import org.webrtc.IceCandidate
import org.webrtc.MediaConstraints
import org.webrtc.MediaStream
import org.webrtc.PeerConnection
import org.webrtc.PeerConnection.IceConnectionState
import org.webrtc.PeerConnection.IceGatheringState
import org.webrtc.PeerConnection.IceServer
import org.webrtc.PeerConnection.RTCConfiguration
import org.webrtc.PeerConnection.SignalingState
import org.webrtc.PeerConnectionFactory
import org.webrtc.PeerConnectionFactory.InitializationOptions
import org.webrtc.RendererCommon
import org.webrtc.RtpReceiver
import org.webrtc.SdpObserver
import org.webrtc.SessionDescription
import org.webrtc.SurfaceViewRenderer
import org.webrtc.VideoDecoderFactory
import org.webrtc.VideoEncoderFactory
import org.webrtc.VideoTrack
import org.webrtc.audio.AudioDeviceModule
import org.webrtc.audio.JavaAudioDeviceModule
import timber.log.Timber
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.net.HttpURLConnection
import java.net.URI
import java.net.URISyntaxException
import java.net.URL
import java.nio.charset.StandardCharsets
import java.util.Arrays
import java.util.concurrent.ExecutionException
import java.util.regex.Pattern


enum class SessionState {
    DISCONNECTED, CONNECTING, CONNECTED
}

class MainActivity : ComponentActivity() {
    // Update below values if you want to use a different avatar
    private val avatarCharacter = "lisa"
    private val avatarStyle = "casual-sitting"

    // Set below parameter to true if you want to use custom avatar
    private val customAvatar = false

    // Update below value if you want to use a different TTS voice
    private val ttsVoice = "en-US-AvaMultilingualNeural"

    // Fill below value if you want to use custom TTS voice
    private val ttsEndpointID = ""

    private val VIDEO_TRACK_ID = "ARDAMSv0"
    private val AUDIO_TRACK_ID = "ARDAMSa0"
    private val AUDIO_CODEC_ISAC = "ISAC"
    private val VIDEO_CODEC_H264 = "H264"

    private var iceUrl: String? = null
    private var iceUsername: String? = null
    private var icePassword: String? = null

    private var speechConfig: SpeechConfig? = null
    private var synthesizer: SpeechSynthesizer? = null
    private var connection: Connection? = null

    private var peerConnectionFactory: PeerConnectionFactory? = null
    private var peerConnection: PeerConnection? = null
    private var audioTrack: AudioTrack? = null
    private var videoTrack: VideoTrack? = null
    private var sdpObserver: SdpObserver? = null

    private var videoRenderer: SurfaceViewRenderer? = null
    private var frameListener: EglRenderer.FrameListener? = null

    companion object {
        private var loaded = false
    }

    data class MainActivityState(
        val enableAzure: Boolean = false,
        val openaiSessionState: OpenAIState = OpenAIState.DISCONNECTED,
        val avatarSessionState: SessionState = SessionState.DISCONNECTED,
        val isSpeaking: Boolean = false,
        val aiResponseText: String = ""
    )
    private val state = MutableStateFlow(MainActivityState())

    private val audioInput = AudioInput()
    private val openai = OpenAI()

    private var webView: WebView? = null

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            AIAppTheme {
//                var backEnabled by remember { mutableStateOf(false) }
//                BackHandler(enabled = true) {
//                    webView?.goBack()
//                }

                val permissionResultLauncher = rememberLauncherForActivityResult(
                    contract = ActivityResultContracts.RequestPermission(),
                    onResult = {}
                )

                LaunchedEffect(Unit) {
                    if (ActivityCompat.checkSelfPermission(
                            this@MainActivity,
                            Manifest.permission.RECORD_AUDIO
                        ) != PackageManager.PERMISSION_GRANTED
                    ) {
                        permissionResultLauncher.launch(Manifest.permission.RECORD_AUDIO)
                    }
                }

                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    val drawerState = rememberDrawerState(initialValue = DrawerValue.Closed)
                    val scope = rememberCoroutineScope()

                    Box(
                        modifier = Modifier
                            .padding(innerPadding)
                            .fillMaxSize()
                            .pointerInput(Unit) {
                                detectHorizontalDragGestures(
                                    onDragStart = { offset ->
                                        // Check if the gesture started near the edge of the screen
                                        if (offset.x < 100f && drawerState.isClosed) {
                                            scope.launch { drawerState.open() }
                                        }
                                    },
                                    onDragEnd = {
                                        // Optional: Close the drawer if needed
                                        if (drawerState.isOpen) {
                                            scope.launch { drawerState.close() }
                                        }
                                    },
                                    onDragCancel = {},
                                    onHorizontalDrag = { _, _ -> }
                                )
                            }
                    ) {
                        val state by state.collectAsState()

                        ModalNavigationDrawer(
                            drawerState = drawerState,
                            drawerContent = {
                                ModalDrawerSheet {
                                    AIBotContent(
                                        state = state,
                                        onAzureEnabledToggle = {
                                            this@MainActivity.state.value = state.copy(enableAzure = it)
                                        },
                                        onVideoRendererCreated = {
                                            Timber.i("Video renderer created")
                                            videoRenderer = it
                                            initializePeerConnectionFactory()
                                        },
                                        onStartSessionButtonClicked = ::onStartSessionButtonClicked,
                                        onStopSessionButtonClicked = ::onStopSessionButtonClicked
                                    )
                                }
                            },
                            gesturesEnabled = false
                        ) {
                            Box(
                                modifier = Modifier.fillMaxSize(),
                                contentAlignment = Alignment.Center
                            ) {
                                Button(
                                    onClick = ::openWebsite
                                ) {
                                    Text("Open Foxglove")
                                }
                            }

//                            // Remember the WebView instance across recompositions
//                            val context = LocalContext.current
//                            val webView = remember { WebView(context) }
//                            var loaded by remember { mutableStateOf(false) }

//                            PullToRefresh(
//                                onRefresh = { onComplete ->
//                                    loaded = false
//                                    webView.loadUrl(webViewUrl)
//                                    lifecycleScope.launch(Dispatchers.IO) {
//                                        while (!loaded) {
//                                            delay(100)
//                                        }
//                                        onComplete()
//                                    }
//                                }
//                            ) {
//                                LazyColumn(
//                                    modifier = Modifier
//                                        .fillMaxSize(),
//                                ) {
//                                    item {
//                                        // Configure and manage the WebView state
//                                        DisposableEffect(Unit) {
//                                            this@MainActivity.webView = webView
//
//                                            webView.layoutParams = ViewGroup.LayoutParams(
//                                                ViewGroup.LayoutParams.MATCH_PARENT,
//                                                ViewGroup.LayoutParams.MATCH_PARENT
//                                            )
//
//                                            webView.settings.apply {
//                                                javaScriptEnabled = true
//                                                allowFileAccess = true
//                                                allowContentAccess = true
//                                                domStorageEnabled = true
//                                                mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW
//
//                                                loadWithOverviewMode = true
//                                                useWideViewPort = true
//                                                builtInZoomControls = true
//                                                displayZoomControls = false
//
//                                                javaScriptEnabled = true
//                                                domStorageEnabled = true
//                                                javaScriptCanOpenWindowsAutomatically = true
////                                                setSupportMultipleWindows(false)
//
//                                                pluginState = WebSettings.PluginState.ON
//                                                cacheMode = WebSettings.LOAD_DEFAULT
//
//                                                javaScriptEnabled = true
//                                                domStorageEnabled = true
//                                                databaseEnabled = true
//                                                javaScriptCanOpenWindowsAutomatically = true
//                                                setSupportMultipleWindows(false)
////                                                setAppCacheEnabled(true)
//                                                allowFileAccess = true
//                                                allowContentAccess = true
//                                                mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW
//                                                loadWithOverviewMode = true
//                                                useWideViewPort = true
//                                            }
//
//                                            // enable debugging
//                                            WebView.setWebContentsDebuggingEnabled(true)
//
//                                            // enable cookies
//                                            CookieManager.getInstance().setAcceptCookie(true)
//                                            CookieManager.getInstance().setAcceptThirdPartyCookies(webView, true)
//
//                                            // Define a JavaScript interface
//                                            class WebAppInterface(private val context: Context) {
//                                            }
//
//                                            // Add the JavaScript interface to your WebView
//                                            webView.addJavascriptInterface(WebAppInterface(context), "Android")
//
//                                            val assetLoader = WebViewAssetLoader.Builder()
//                                                .addPathHandler(
//                                                    "/assets/",
//                                                    WebViewAssetLoader.AssetsPathHandler(context)
//                                                )
//                                                .build()
//
//                                            class CustomWebChromeClient : WebChromeClient() {
////                                                override fun onCreateWindow(view: WebView?, isDialog: Boolean, isUserGesture: Boolean, resultMsg: Message?): Boolean {
////                                                    val newWebView = WebView(view?.context ?: return false).apply {
////                                                        settings.javaScriptEnabled = true
////                                                    }
////                                                    val dialog = Dialog(view.context).apply {
////                                                        setContentView(newWebView)
////                                                        show()
////                                                    }
////                                                    newWebView.webViewClient = WebViewClient()
////                                                    resultMsg?.sendToTarget()
////                                                    return true
////                                                }
//                                            }
//
//                                            webView.webChromeClient = CustomWebChromeClient()
//                                            webView.getSettings().userAgentString = "Android WebView"
//
//
//                                            webView.webViewClient = object : WebViewClient() {
//                                                override fun onPageFinished(view: WebView?, url: String?) {
//                                                    loaded = true
//                                                    super.onPageFinished(view, url)
//                                                }
//
//                                                override fun onPageStarted(view: WebView, url: String?, favicon: Bitmap?) {
//                                                    backEnabled = view.canGoBack()
//                                                }
//
//                                                @SuppressLint("WebViewClientOnReceivedSslError")
//                                                override fun onReceivedSslError(
//                                                    view: WebView,
//                                                    handler: SslErrorHandler,
//                                                    error: SslError
//                                                ) {
//                                                    handler.proceed() // Only for debugging; do not use in production
//                                                }
//
////                                                @Deprecated("Deprecated in Java")
////                                                override fun shouldOverrideUrlLoading(view: WebView, url: String?): Boolean {
////                                                    return false  // Let WebView handle all redirects
////                                                }
//
//                                                @Deprecated("Deprecated in Java")
//                                                override fun shouldOverrideUrlLoading(
//                                                    view: WebView,
//                                                    url: String?
//                                                ): Boolean {
//                                                    Timber.i("webview url to load: $url")
//
////                                                    if (url == "https://app.foxglove.dev/signin?returnTo=%2F200") {
////                                                        view.loadUrl(webViewUrl)
////                                                    } else {
//                                                        view.loadUrl(url!!)
////                                                    }
//
//                                                    return true
//                                                }
////
////                                                override fun shouldOverrideUrlLoading(
////                                                    view: WebView,
////                                                    request: WebResourceRequest
////                                                ): Boolean {
////                                                    view.loadUrl(request.url.toString())
////                                                    Timber.i("loading: build.VERSION_CODES.N")
////                                                    return true
////                                                    //return super.shouldOverrideUrlLoading(view, request);
////                                                }
//
//                                                @Deprecated(
//                                                    "Deprecated in Java", ReplaceWith(
//                                                        "assetLoader.shouldInterceptRequest(Uri.parse(url))!!",
//                                                        "android.net.Uri"
//                                                    )
//                                                )
//                                                override fun shouldInterceptRequest(
//                                                    view: WebView?,
//                                                    url: String?
//                                                ): WebResourceResponse {
//                                                    return assetLoader.shouldInterceptRequest(Uri.parse(url))!!
//                                                }
//
//                                                override fun shouldInterceptRequest(
//                                                    view: WebView?,
//                                                    request: WebResourceRequest?
//                                                ): WebResourceResponse? {
//                                                    return assetLoader.shouldInterceptRequest(request!!.url)
//                                                }
//                                            }
//
//                                            // Restore state or load URL
//                                            if (savedInstanceState != null) {
//                                                webView.restoreState(savedInstanceState)
//                                            } else {
//                                                Timber.i("loading url start: $webViewUrl")
//                                                webView.loadUrl(webViewUrl)
//                                            }
//
//                                            onDispose { }
//                                        }
//
//                                        AndroidView(
//                                            modifier = Modifier
//                                                .aspectRatio(1f)
////                                                .width(320.dp)
////                                                .height(540.dp)
//                                                .padding(16.dp),
//                                            factory = { webView },
////                                            update = {
////                                                it.loadUrl(webViewUrl)
////                                            }
//                                        )
//                                    }
//                                }
//                            }
                        }
                    }
                }
            }
        }

        val audioManager = getSystemService(AUDIO_SERVICE) as AudioManager
        audioManager.mode = AudioManager.MODE_IN_COMMUNICATION // will make sound bar show for phone instead of media, but will activate echo cancellation
        audioManager.isSpeakerphoneOn = true // Switch audio output device from earphone to speaker, for louder volume.

        lifecycleScope.launch(Dispatchers.IO) {
            while(true) {
                val base64Data = audioInput.read()
                if (base64Data != null) {
                    if (/*state.value.avatarSessionState == SessionState.CONNECTED &&*/ state.value.openaiSessionState == OpenAIState.CONNECTED) {
                        openai.sendInputAudioToWebsocket(base64Data)
                    }
                }
            }
        }

        if (!loaded) {
            loaded = true
            openWebsite()
        }
    }

    private fun openWebsite() {
        // open https://app.foxglove.dev/ in chrome
        // launch in adjacent mode: https://developer.android.com/develop/ui/views/layout/support-multi-window-mode#launch_adjacent
        val intent = Intent(
            Intent.ACTION_VIEW,
            Uri.parse(webViewUrl)
        )
        intent.setFlags(Intent.FLAG_ACTIVITY_LAUNCH_ADJACENT or Intent.FLAG_ACTIVITY_NEW_TASK)
        startActivity(intent)
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        webView?.saveState(outState) // Save WebView state
    }

    override fun onRestoreInstanceState(savedInstanceState: Bundle) {
        super.onRestoreInstanceState(savedInstanceState)
        webView?.restoreState(savedInstanceState) // Restore WebView state
    }

    // region OpenAI
    fun startOpenAI() {
        audioInput.startRecording()

        lifecycleScope.launch(Dispatchers.IO) {
            openai.startWebsocket(
                onSessionState = { newState ->
                    state.value = state.value.copy(openaiSessionState = newState)
                },
                onUserTalking = {
                    // stop talking
                    onStopSpeakingButtonClicked()

                    state.value = state.value.copy(aiResponseText = "")
                },
                onBotDeltaResponse = { response ->
                    state.value = state.value.copy(aiResponseText = state.value.aiResponseText + response)
                },
                onBotResponse = { response ->
                    // start talking
                    onSpeakButtonClicked(response)
                }
            )
        }
    }

    fun stopOpenAI() {
        audioInput.close()
        openai.stopWebsocket()
    }
    // endregion

    override fun onDestroy() {
        super.onDestroy()

        // Release speech synthesizer and its dependencies
        if (synthesizer != null) {
            synthesizer!!.close()
            connection!!.close()
        }

        if (speechConfig != null) {
            speechConfig!!.close()
        }

        if (peerConnection != null) {
            peerConnection!!.close()
        }
    }

    @Throws(URISyntaxException::class)
    fun onStartSessionButtonClicked() {
        if (state.value.enableAzure) {
            state.value = state.value.copy(avatarSessionState = SessionState.CONNECTING)

            if (synthesizer != null) {
                speechConfig!!.close()
                synthesizer!!.close()
                connection!!.close()
            }

            if (peerConnection != null) {
                peerConnection!!.close()
            }

            videoRenderer!!.addFrameListener(frameListener, 1.0f)

            fetchIceToken()
        }

        startOpenAI()
    }

    @Throws(ExecutionException::class, InterruptedException::class)
    fun onSpeakButtonClicked(text: String) {
        if (synthesizer == null) {
            updateOutputMessage("Please start the avatar session first", true, true)
            return
        }

        onStopSpeakingButtonClicked()

        state.value = state.value.copy(isSpeaking = true)

        synthesizer!!.SpeakTextAsync(text)
    }

    fun onStopSpeakingButtonClicked() {
        if (state.value.isSpeaking) {
            connection!!.sendMessageAsync("synthesis.control", "{\"action\":\"stop\"}")
        }

        state.value = state.value.copy(isSpeaking = false)
    }

    fun onStopSessionButtonClicked() {
        if (synthesizer == null) {
            updateOutputMessage("Please start the avatar session first", true, true)
        } else {
            synthesizer!!.close()
        }

        stopOpenAI()
    }

    private fun initializePeerConnectionFactory() {
        // Prepare peer connection factory
        val initializationOptions =
            InitializationOptions
                .builder(applicationContext)
                .setEnableInternalTracer(true)
                .createInitializationOptions()
        PeerConnectionFactory.initialize(initializationOptions)

        val options = PeerConnectionFactory.Options()
        val eglBase = EglBase.create()
        val videoDecoderFactory: VideoDecoderFactory =
            DefaultVideoDecoderFactory(eglBase.eglBaseContext)
        val videoEncoderFactory: VideoEncoderFactory =
            DefaultVideoEncoderFactory(eglBase.eglBaseContext, true, true)
        val audioDeviceModule: AudioDeviceModule =
            JavaAudioDeviceModule.builder(this).createAudioDeviceModule()
        peerConnectionFactory = PeerConnectionFactory.builder()
            .setOptions(options)
            .setVideoDecoderFactory(videoDecoderFactory)
            .setVideoEncoderFactory(videoEncoderFactory)
            .setAudioDeviceModule(audioDeviceModule)
            .createPeerConnectionFactory()

        // Initialize video renderer
        videoRenderer!!.init(eglBase.eglBaseContext, null)
        videoRenderer!!.setScalingType(RendererCommon.ScalingType.SCALE_ASPECT_FILL)
//        setVideoRendererVisibility(false)
        frameListener = EglRenderer.FrameListener { bitmap: Bitmap? ->
//            setVideoRendererVisibility(true)
            synthesizer!!.SynthesisStarted.addEventListener { o: Any?, e: SpeechSynthesisEventArgs ->
                state.value = state.value.copy(isSpeaking = true)
                e.close()
            }
            synthesizer!!.SynthesisCompleted.addEventListener { o: Any?, e: SpeechSynthesisEventArgs ->
                state.value = state.value.copy(isSpeaking = false)
                e.close()
            }
        }

        // Create audio track
        val audioConstraints = MediaConstraints()
        audioConstraints.mandatory.add(
            MediaConstraints.KeyValuePair(
                "googEchoCancellation",
                "true"
            )
        )
        audioConstraints.mandatory.add(MediaConstraints.KeyValuePair("googAutoGainControl", "true"))
        audioConstraints.mandatory.add(MediaConstraints.KeyValuePair("googHighpassFilter", "true"))
        audioConstraints.mandatory.add(
            MediaConstraints.KeyValuePair(
                "googNoiseSuppression",
                "true"
            )
        )
        val audioSource = peerConnectionFactory!!.createAudioSource(audioConstraints)
        audioTrack = peerConnectionFactory!!.createAudioTrack(AUDIO_TRACK_ID, audioSource)

        // Create video track
        val videoSource = peerConnectionFactory!!.createVideoSource(false)
        videoTrack = peerConnectionFactory!!.createVideoTrack(VIDEO_TRACK_ID, videoSource)
    }

    private fun fetchIceToken() {
        val runnable = Runnable {
            if (iceUrl == null) {
                updateOutputMessage("Fetching ICE token ...", false, true)
                try {
                    val endpoint =
                        "https://" + speechServiceRegion + ".tts.speech.microsoft.com"
                    val url = URL("$endpoint/cognitiveservices/avatar/relay/token/v1")
                    val urlConnection = url.openConnection() as HttpURLConnection
                    urlConnection.requestMethod = "GET"
                    urlConnection.setRequestProperty(
                        "Ocp-Apim-Subscription-Key",
                        speechServiceKey
                    )
                    urlConnection.connect()
                    val responseStream = urlConnection.inputStream
                    val streamReader = InputStreamReader(responseStream)
                    val bufferedReader = BufferedReader(streamReader)
                    val responseStringBuilder = StringBuilder()
                    var responseLine: String?
                    while ((bufferedReader.readLine().also { responseLine = it }) != null) {
                        responseStringBuilder.append(responseLine)
                    }

                    val responseString = responseStringBuilder.toString()
                    val iceTokenJsonObj = JSONObject(responseString)
                    iceUrl = iceTokenJsonObj.getJSONArray("Urls").getString(0)
                    iceUsername = iceTokenJsonObj.getString("Username")
                    icePassword = iceTokenJsonObj.getString("Password")
                    updateOutputMessage("ICE token successfully fetched", false, true)
                    setupWebRTC()
                } catch (e: IOException) {
                    Timber.tag("[ICE Token]").e(e.toString())
                } catch (e: JSONException) {
                    Timber.tag("[ICE Token]").e(e.toString())
                }
            } else {
                setupWebRTC()
            }
        }

        val thread = Thread(runnable)
        thread.start()
    }

    private fun setupWebRTC() {
        // Create peer connection
        val iceServers: MutableList<IceServer> = ArrayList()
        val iceServer = IceServer
            .builder(iceUrl)
            .setUsername(iceUsername)
            .setPassword(icePassword)
            .createIceServer()
        iceServers.add(iceServer)

        val rtcConfig = RTCConfiguration(iceServers)
        rtcConfig.continualGatheringPolicy =
            PeerConnection.ContinualGatheringPolicy.GATHER_CONTINUALLY
        rtcConfig.iceTransportsType = PeerConnection.IceTransportsType.RELAY
        rtcConfig.audioJitterBufferMaxPackets = 0

        // Encode SDP and send to service
        val peerConnectionObserver: PeerConnection.Observer = object : PeerConnection.Observer {
            private var iceCandidateCount = 0

            override fun onSignalingChange(signalingState: SignalingState) {
            }

            override fun onIceConnectionChange(iceConnectionState: IceConnectionState) {
                Timber.tag("[WebRTC][PeerConnectionObserver]")
                    .i("WebRTC connection state: " + iceConnectionState.name)
                updateOutputMessage(
                    "WebRTC connection state: " + iceConnectionState.name,
                    false,
                    true
                )
                if (iceConnectionState == IceConnectionState.CONNECTED) {
                    state.value = state.value.copy(avatarSessionState = SessionState.CONNECTED)
                } else if (iceConnectionState == IceConnectionState.DISCONNECTED || iceConnectionState == IceConnectionState.FAILED) {
//                    setVideoRendererVisibility(false)
                    state.value = state.value.copy(
                        avatarSessionState = SessionState.DISCONNECTED,
                        isSpeaking = false
                    )
                }
            }

            override fun onIceConnectionReceivingChange(b: Boolean) {
            }

            override fun onIceGatheringChange(iceGatheringState: IceGatheringState) {
                Timber.tag("[WebRTC][PeerConnectionObserver]")
                    .i("ICE gathering state: " + iceGatheringState.name)
                updateOutputMessage("ICE gathering state: " + iceGatheringState.name, false, true)
            }

            override fun onIceCandidate(iceCandidate: IceCandidate) {
                iceCandidateCount++
                Timber.tag("[WebRTC][PeerConnectionObserver]").i("ICE candidate: $iceCandidate")
                if (iceCandidateCount >= 2) {
                    // Encode SDP and send to service
                    val sdpJsonObj = JSONObject()
                    try {
                        sdpJsonObj.put("type", "offer")
                        sdpJsonObj.put("sdp", peerConnection!!.localDescription.description)
                    } catch (e: JSONException) {
                        throw RuntimeException(e)
                    }

                    val sdpJsonStr = sdpJsonObj.toString()
                    val sdpBase64Str =
                        Base64.encodeToString(sdpJsonStr.toByteArray(), Base64.NO_WRAP)
                    connectAvatar(sdpBase64Str)
                }
            }

            override fun onIceCandidatesRemoved(iceCandidates: Array<IceCandidate>) {
            }

            override fun onAddStream(mediaStream: MediaStream) {
            }

            override fun onRemoveStream(mediaStream: MediaStream) {
            }

            override fun onDataChannel(dataChannel: DataChannel) {
            }

            override fun onRenegotiationNeeded() {
            }

            override fun onAddTrack(rtpReceiver: RtpReceiver, mediaStreams: Array<MediaStream>) {
                if (mediaStreams.size > 0) {
                    if (!mediaStreams[0].videoTracks.isEmpty()) {
                        mediaStreams[0].videoTracks[0].addSink(videoRenderer)
                    }

                    if (!mediaStreams[0].audioTracks.isEmpty()) {
                        mediaStreams[0].audioTracks[0].setEnabled(true)

                        // To boost the audio volume, set below value to be greater than 1.0
                        mediaStreams[0].audioTracks[0].setVolume(1.0)
                    }
                }
            }
        }

        // Create peer connection
        peerConnection =
            peerConnectionFactory!!.createPeerConnection(rtcConfig, peerConnectionObserver)
        peerConnection!!.addTrack(audioTrack)
        peerConnection!!.addTrack(videoTrack)

        // Add offer constrains
        val constraints = MediaConstraints()
        constraints.mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "true"))
        constraints.mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "true"))

        // Create offer
        sdpObserver = object : SdpObserver {
            override fun onCreateSuccess(sessionDescription: SessionDescription) {
                var sdp = sessionDescription.description
                sdp = preferCodec(sdp, AUDIO_CODEC_ISAC, true)
                sdp = preferCodec(sdp, VIDEO_CODEC_H264, false)
                peerConnection!!.setLocalDescription(
                    sdpObserver,
                    SessionDescription(sessionDescription.type, sdp)
                )
                Timber.tag("[WebRTC][SdpObserver]")
                    .i("Offer creation succeeded: " + sessionDescription.description)
            }

            override fun onSetSuccess() {
            }

            override fun onCreateFailure(s: String) {
            }

            override fun onSetFailure(s: String) {
            }
        }
        peerConnection!!.createOffer(sdpObserver, constraints)
    }

    private fun connectAvatar(localSDP: String) {
        val endpoint = "wss://" + speechServiceRegion + ".tts.speech.microsoft.com"
        val uri = URI.create("$endpoint/cognitiveservices/websocket/v1?enableTalkingAvatar=true")
        speechConfig = SpeechConfig.fromEndpoint(uri, speechServiceKey)
        speechConfig!!.setSpeechSynthesisVoiceName(ttsVoice)
        if (ttsEndpointID.isNotEmpty()) {
            speechConfig!!.setEndpointId(ttsEndpointID)
        }

        synthesizer = SpeechSynthesizer(speechConfig, null)
        connection = Connection.fromSpeechSynthesizer(synthesizer)
        synthesizer!!.SynthesisCanceled.addEventListener { o: Any?, e: SpeechSynthesisEventArgs ->
            val cancellationDetails =
                SpeechSynthesisCancellationDetails.fromResult(e.result).toString()
            updateOutputMessage(cancellationDetails, true, true)
            Timber.tag("[Synthesizer]").e(cancellationDetails)
            e.close()
        }

        val avatarConfig = buildAvatarConfig(localSDP)
        try {
            val avatarConfigJsonObj = JSONObject(avatarConfig)
        } catch (e: JSONException) {
            Timber.tag("[WebRTC]").e(e.toString())
        }

        connection!!.setMessageProperty("speech.config", "context", avatarConfig)
        synthesizer!!.SpeakText("")
        val turnStartMessage =
            synthesizer!!.properties.getProperty("SpeechSDKInternal-ExtraTurnStartMessage")
        try {
            val turnStartMessageJsonObj = JSONObject(turnStartMessage)
            val remoteSdpBase64 =
                turnStartMessageJsonObj.getJSONObject("webrtc").getString("connectionString")
            val remoteSdpJsonStr =
                String(Base64.decode(remoteSdpBase64, Base64.DEFAULT), StandardCharsets.UTF_8)
            val remoteSdpJsonObj = JSONObject(remoteSdpJsonStr)
            val remoteSdp = remoteSdpJsonObj.getString("sdp")
            Timber.tag("[WebRTC][Remote SDP]").i("Remote SDP: $remoteSdp")
            peerConnection!!.setRemoteDescription(
                sdpObserver,
                SessionDescription(SessionDescription.Type.ANSWER, remoteSdp)
            )
        } catch (e: JSONException) {
            Timber.tag("[WebRTC][Remote SDP]").e(e.toString())
        }
    }
//    @Synchronized
//    private fun setVideoRendererVisibility(visible: Boolean) {
//        this.runOnUiThread {
//            if (visible) {
//                videoRenderer!!.setBackgroundColor(0x00000000)
//            } else {
//                videoRenderer!!.setBackgroundColor(-0x1)
//            }
//        }
//    }

    @Synchronized
    private fun setButtonAvailability(button: Button, enabled: Boolean) {
        this.runOnUiThread {
            button.isEnabled = enabled
        }
    }

    @Synchronized
    private fun updateOutputMessage(text: String, error: Boolean, append: Boolean) {
        if (error) {
            Timber.e(text)
        } else {
            Timber.i(text)
        }
    }

    private fun buildAvatarConfig(localSDP: String): String {
        return """{
            "synthesis": {
                "video": {
                    "protocol": {
                        "name": "WebRTC",
                        "webrtcConfig": {
                            "clientDescription": "$localSDP",
                            "iceServers": [{
                                "urls": [ "$iceUrl" ],
                                "username": "$iceUsername",
                                "credential": "$icePassword"
                            }]
                        }
                    },
                    "format":{
                        "crop":{
                            "topLeft":{
                                "x": 640,
                                "y": 0
                            },
                            "bottomRight":{
                                "x": 1280,
                                "y": 1080
                            }
                        },
                        "bitrate": 1000000
                    },
                    "talkingAvatar": {
                        "customized": $customAvatar,
                        "character": "$avatarCharacter",
                        "style": "$avatarStyle",
                        "background": {
                            "color": "#FFFFFFFF",
                            "image": {
                                "url": ""
                            }
                        }
                    }
                }
            }
        }"""
    }

    private fun preferCodec(sdp: String, codec: String, isAudio: Boolean): String {
        val lines = sdp.split("\r\n".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
        val mLineIndex = findMediaDescriptionLine(isAudio, lines)
        if (mLineIndex == -1) {
            Timber.tag("[WebRTC][Local SDP]").w("No mediaDescription line, so can't prefer $codec")
            return sdp
        }

        // A list with all the payload types with name `codec`. The payload types are integers in the
        // range 96-127, but they are stored as strings here.
        val codecPayloadTypes: MutableList<String?> = ArrayList()
        // a=rtpmap:<payload type> <encoding name>/<clock rate> [/<encoding parameters>]
        val codecPattern = Pattern.compile("^a=rtpmap:(\\d+) $codec(/\\d+)+[\r]?$")
        for (line in lines) {
            val codecMatcher = codecPattern.matcher(line)
            if (codecMatcher.matches()) {
                codecPayloadTypes.add(codecMatcher.group(1))
            }
        }

        if (codecPayloadTypes.isEmpty()) {
            Timber.tag("[WebRTC][Local SDP]").w("No payload types with name $codec")
            return sdp
        }

        val newMLine =
            movePayloadTypesToFront(codecPayloadTypes, lines[mLineIndex]) ?: return sdp

        Timber.tag("[WebRTC][Local SDP]")
            .d("Change media description from: " + lines[mLineIndex] + " to " + newMLine)
        lines[mLineIndex] = newMLine
        return joinString(Arrays.asList(*lines), "\r\n", true /* delimiterAtEnd */)
    }

    // Returns the line number containing "m=audio|video", or -1 if no such line exists.
    private fun findMediaDescriptionLine(isAudio: Boolean, sdpLines: Array<String>): Int {
        val mediaDescription = if (isAudio) "m=audio " else "m=video "
        for (i in sdpLines.indices) {
            if (sdpLines[i].startsWith(mediaDescription)) {
                return i
            }
        }
        return -1
    }

    private fun movePayloadTypesToFront(
        preferredPayloadTypes: List<String?>, mLine: String
    ): String? {
        // The format of the media description line should be: m=<media> <port> <proto> <fmt> ...
        val origLineParts =
            Arrays.asList(*mLine.split(" ".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray())
        if (origLineParts.size <= 3) {
            Timber.tag("[WebRTC][Local SDP]").e("Wrong SDP media description format: $mLine")
            return null
        }
        val header: List<String?> = origLineParts.subList(0, 3)
        val unpreferredPayloadTypes: MutableList<String?> =
            ArrayList(origLineParts.subList(3, origLineParts.size))
        unpreferredPayloadTypes.removeAll(preferredPayloadTypes)
        // Reconstruct the line with `preferredPayloadTypes` moved to the beginning of the payload
        // types.
        val newLineParts: MutableList<String?> = ArrayList()
        newLineParts.addAll(header)
        newLineParts.addAll(preferredPayloadTypes)
        newLineParts.addAll(unpreferredPayloadTypes)
        return joinString(newLineParts, " ", false /* delimiterAtEnd */)
    }

    private fun joinString(
        s: Iterable<CharSequence?>, delimiter: String, delimiterAtEnd: Boolean
    ): String {
        val iter = s.iterator()
        if (!iter.hasNext()) {
            return ""
        }

        val buffer = StringBuilder(iter.next())
        while (iter.hasNext()) {
            buffer.append(delimiter).append(iter.next())
        }

        if (delimiterAtEnd) {
            buffer.append(delimiter)
        }

        return buffer.toString()
    }
}

@Composable
fun AIBotContent(
    state: MainActivity.MainActivityState,
    onAzureEnabledToggle: (Boolean) -> Unit,
    onVideoRendererCreated: (SurfaceViewRenderer) -> Unit,
    onStartSessionButtonClicked: () -> Unit,
    onStopSessionButtonClicked: () -> Unit
) {
    val context = LocalContext.current

    Column(
        modifier = Modifier
            .widthIn(max = 500.dp)
            .fillMaxWidth()
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Button(
            modifier = Modifier.padding(top = 16.dp),
            onClick = {
                if ((!state.enableAzure || state.avatarSessionState == SessionState.DISCONNECTED) && state.openaiSessionState == OpenAIState.DISCONNECTED) {
                    onStartSessionButtonClicked()
                } else {
                    onStopSessionButtonClicked()
                }
            }
        ) {
            Text(
                text = if ((!state.enableAzure || state.avatarSessionState == SessionState.DISCONNECTED) && state.openaiSessionState == OpenAIState.DISCONNECTED) {
                    "Start Session"
                } else {
                    "Stop Session"
                }
            )
        }

        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Text(
                text = "Enable Azure",
            )
            Switch(
                checked = state.enableAzure,
                onCheckedChange = onAzureEnabledToggle
            )
        }

        Text("Azure: ${state.avatarSessionState}")
        Text("OpenAI: ${state.openaiSessionState}")

        BoxWithConstraints(
            modifier = Modifier.fillMaxSize()
        ) {
            // Aspect ratio (e.g., 320:540)
            val aspectRatio = 320f / 540f

            // Calculate width, constrained to a maximum of 200.dp
            val maxWidthDp = 200.dp
            val width = if (constraints.maxWidth.dp > maxWidthDp) maxWidthDp else constraints.maxWidth.dp
            val height = width / aspectRatio

            AndroidView(
                modifier = Modifier
                    .width(width)
                    .height(height),
                factory = {
                    SurfaceViewRenderer(context).apply {
                        onVideoRendererCreated(this)
                    }
                }
            )
        }

        Text("Response:\n${state.aiResponseText}")
    }
}

@Preview(showBackground = true, widthDp = 1000, heightDp = 700)
@Composable
fun AIBotContentPreview() {
    AIAppTheme {
        AIBotContent(
            state = MainActivity.MainActivityState(),
            onAzureEnabledToggle = {},
            onVideoRendererCreated = {},
            onStartSessionButtonClicked = {},
            onStopSessionButtonClicked = {}
        )
    }
}