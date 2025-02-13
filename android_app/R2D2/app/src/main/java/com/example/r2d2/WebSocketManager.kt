package com.example.r2d2
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.widget.Toast
import kotlinx.coroutines.*
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

class WebSocketManager(private val url: String, val sharedState: SharedState, private val context: Context) {
    private var client: OkHttpClient = OkHttpClient()
    private lateinit var webSocket: WebSocket

    private var lastCommand: String? = null
    private var lastCommandTimestamp: Long = 0
    private val resendDelay = 100L
    private var resendJob: Job? = null

    // WebSocket connection
    fun connectToWebSocket() {
        val request = Request.Builder().url(url).build()

        val maxMessages = 100

        webSocket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                showToast("Connected to WebSocket")
                sharedState.isConnected = true
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                if (sharedState.receivedMessages.size >= maxMessages) {
                    sharedState.receivedMessages = sharedState.receivedMessages.drop(1) + text
                } else {
                    sharedState.receivedMessages += text
                }
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                showToast("Error: ${t.message}")
                sharedState.isConnected = false
                reconnectWebSocket()
            }

            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                super.onClosing(webSocket, code, reason)
                showToast("Connection closed!")
                sharedState.isConnected = false
            }
        })
        startResendChecker()
    }

    private fun showToast(message: String) {
        Handler(Looper.getMainLooper()).post {
            Toast.makeText(context, message, Toast.LENGTH_SHORT).show()
        }
    }

    fun reconnectWebSocket() {
        Handler(Looper.getMainLooper()).postDelayed({
            showToast("Trying to reconnect...")
            connectToWebSocket()
        }, 5000)
    }

    fun sendMovementCommand(x: Byte, y: Byte) {
        val command = "move $x,$y"
        sendCommand(command)
    }

    fun sendCommandToRobot(command: String) {
        sendCommand(command)
    }

    private fun sendCommand(command: String) {
        if (::webSocket.isInitialized) {
            webSocket.send(command)
            updateLastCommand(command)
        } else {
            showToast("WebSocket not connected!")
        }
    }

    private fun updateLastCommand(command: String) {
        lastCommand = command
        lastCommandTimestamp = System.currentTimeMillis()
    }

    private fun startResendChecker() {
        resendJob = CoroutineScope(Dispatchers.IO).launch {
            while (true) {
                val currentTime = System.currentTimeMillis()
                if (lastCommand != null && (currentTime - lastCommandTimestamp > resendDelay)) {
                    lastCommand?.let { command ->
                        webSocket.send(command)
                        updateLastCommand(command)
                    }
                }
                delay(resendDelay)
            }
        }
    }
}