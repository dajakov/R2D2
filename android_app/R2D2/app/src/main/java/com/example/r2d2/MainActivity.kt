package com.example.r2d2

import android.annotation.SuppressLint
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.hapticfeedback.HapticFeedbackType
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.platform.LocalHapticFeedback
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

private val R2D2Blue = Color(0xFF005AA7)
private val R2D2Gray = Color(0xFF8A8D8F)
private val R2D2White = Color(0xFFFFFFFF)

val R2D2Theme = lightColorScheme(
    primary = R2D2Blue,
    secondary = R2D2Gray,
    background = R2D2White
)

@Composable
fun R2D2AppTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = R2D2Theme,
        typography = Typography(),
        shapes = Shapes(),
        content = {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .background(Color(0xFF212121))
            ) {
                content()
            }
        }
    )
}

class SharedState {
    var isConnected by mutableStateOf(false)
    var receivedMessages by mutableStateOf(listOf<String>())
}

class MainActivity : ComponentActivity() {
    private val sharedState = SharedState()
    private lateinit var webSocketManager: WebSocketManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        webSocketManager = WebSocketManager("ws://192.168.4.1:81", sharedState, this)
        webSocketManager.connectToWebSocket()

        setContent {
            R2D2AppTheme {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(16.dp),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Text(
                        text = "R2-D2 Control V0.1",
                        style = MaterialTheme.typography.headlineMedium,
                        color = R2D2Blue
                    )
                    Spacer(modifier = Modifier.height(5.dp))

                    Text(
                        text = if (webSocketManager.sharedState.isConnected) "Connected" else "Disconnected",
                        color = if (webSocketManager.sharedState.isConnected) Color.Green else Color.Red
                    )

                    DebuggingMessages(modifier = Modifier.weight(1f),
                        receivedMessages = webSocketManager.sharedState.receivedMessages)

                    Spacer(modifier = Modifier.height(10.dp))

                    ControlButtons { command -> webSocketManager.sendCommandToRobot(command) }

                    Spacer(modifier = Modifier.height(10.dp))

                    Joystick(
                        onMove = { x, y ->
                            webSocketManager.sendMovementCommand(
                                x,
                                y
                            )
                        })
                }
            }
        }
    }

    @Composable
    fun ControlButtons(onCommand: (String) -> Unit) {
        Row(modifier = Modifier.fillMaxWidth()) {
            val interactionSourceLeft = remember { MutableInteractionSource() }
            val interactionSourceRight = remember { MutableInteractionSource() }

            ButtonWithHapticFeedback(
                modifier = Modifier.weight(1f),
                command = "left",
                interactionSource = interactionSourceLeft,
                onCommand = onCommand
            )

            Spacer(modifier = Modifier.width(5.dp))

            ButtonWithHapticFeedback(
                modifier = Modifier.weight(1f),
                command = "right",
                interactionSource = interactionSourceRight,
                onCommand = onCommand
            )
        }
    }

    @Composable
    fun ButtonWithHapticFeedback(
        command: String,
        interactionSource: MutableInteractionSource,
        onCommand: (String) -> Unit,
        modifier: Modifier = Modifier
    ) {
        val isPressed by interactionSource.collectIsPressedAsState()
        val hapticFeedback = LocalHapticFeedback.current
        var lastIsPressed by remember { mutableStateOf(false) }

        Button(
            onClick = { onCommand("stop") },
            interactionSource = interactionSource,
            modifier = modifier
        ) {
            Text("Rotate $command")
            if (isPressed && !lastIsPressed) {
                lastIsPressed = true
                onCommand(command)
                hapticFeedback.performHapticFeedback(HapticFeedbackType.LongPress)
            }
            if (!isPressed) {
                lastIsPressed = false
            }
        }
    }

    @SuppressLint("FrequentlyChangedStateReadInComposition")
    @Composable
    fun DebuggingMessages(receivedMessages: List<String>, modifier: Modifier = Modifier) {
        val listState = rememberLazyListState()

        Box(
            modifier = modifier
                .clip(RoundedCornerShape(5.dp))
                .fillMaxWidth()
                .background(Color(0xFF000000)),
            contentAlignment = Alignment.TopEnd
        ) {
            // Main content
            Column(modifier = Modifier.fillMaxWidth().padding(5.dp)) {
                Text(
                    text = "Debugging Messages:",
                    style = MaterialTheme.typography.headlineMedium,
                    color = MaterialTheme.colorScheme.primary,
                    fontSize = 16.sp,
                    lineHeight = 1.sp
                )
                Box(modifier = Modifier.fillMaxSize()) {
                    LazyColumn(
                        state = listState,
                        modifier = Modifier.fillMaxWidth().padding(end = 12.dp)
                    ) {
                        items(receivedMessages) { message ->
                            Row(
                                modifier = Modifier.fillMaxWidth(),
                                horizontalArrangement = Arrangement.SpaceBetween
                            ) {
                                Text(
                                    text = message,
                                    fontSize = 12.sp,
                                    style = MaterialTheme.typography.bodyLarge,
                                    color = MaterialTheme.colorScheme.inverseOnSurface,
                                    lineHeight = 1.sp
                                )
                                Box(
                                    modifier = Modifier
                                        .size(4.dp, 10.dp)
                                        .background(Color.Green)
                                        .align(Alignment.CenterVertically)
                                )
                            }
                        }
                    }
                    // Scroll indicator
                    val totalItems = receivedMessages.size
                    if (totalItems > 0) {
                        val visibleItemsInfo = listState.layoutInfo.visibleItemsInfo
                        val firstVisibleItemIndex = listState.firstVisibleItemIndex
                        val totalScrollableItems = totalItems - visibleItemsInfo.size

                        val indicatorHeightFraction = visibleItemsInfo.size.toFloat() / totalItems
                        val indicatorOffsetFraction = if (totalScrollableItems > 0) {
                            firstVisibleItemIndex.toFloat() / totalScrollableItems
                        } else {
                            0f
                        }

                        Box(
                            modifier = Modifier
                                .fillMaxHeight()
                                .width(6.dp)
                                .align(Alignment.CenterEnd)
                                .background(Color.DarkGray)
                        ) {
                            Box(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height((indicatorHeightFraction * 200.dp.value).dp)
                                    .offset(y = (indicatorOffsetFraction * 200.dp.value).dp)
                                    .background(Color.Cyan, RoundedCornerShape(3.dp))
                            )
                        }
                    }
                }
            }
        }
    }

    @Composable
    fun Joystick(onMove: (Byte, Byte) -> Unit) {
        val configuration = LocalConfiguration.current
        val screenWidth = configuration.screenWidthDp.dp
        val joystickRadius = 80f
        var offsetX by remember { mutableFloatStateOf(0f) }
        var offsetY by remember { mutableFloatStateOf(0f) }

        Box(
            modifier = Modifier
                .clip(RoundedCornerShape(20.dp))
                .width(screenWidth)
                .height(screenWidth * 0.8f)
                .background(Color(0xFF444444)),
            contentAlignment = Alignment.Center
        ) {
            Canvas(
                modifier = Modifier
                    .size(screenWidth, screenWidth)
                    .pointerInput(Unit) {
                        detectDragGestures(
                            onDragEnd = {
                                offsetX = 0f
                                offsetY = 0f
                                onMove(0, 0) // Send (0, 0) on drag end
                            },
                            onDrag = { _, dragAmount ->
                                // Update offsets while ensuring they are within bounds
                                offsetX = (offsetX + dragAmount.x).coerceIn(
                                    -screenWidth.toPx() / 2 + joystickRadius,
                                    screenWidth.toPx() / 2 - joystickRadius
                                )
                                offsetY = (offsetY + dragAmount.y).coerceIn(
                                    -screenWidth.toPx() / 2 + joystickRadius,
                                    screenWidth.toPx() / 2 - joystickRadius
                                )

                                // Normalize the offsets to the range [-128, 127]
                                val normalizedX = ((offsetX / (screenWidth.toPx() / 2 - joystickRadius)) * 127).toInt().coerceIn(-128, 127).toByte()
                                val normalizedY = ((offsetY / (screenWidth.toPx() / 2 - joystickRadius)) * 127).toInt().coerceIn(-128, 127).toByte()

                                // Pass normalized values to the callback
                                onMove(normalizedX, normalizedY)
                            }
                        )
                    }
            ) {
                val canvasCenter = Offset(size.width / 2, size.height / 2)
                drawCircle(
                    color = R2D2Blue,
                    radius = joystickRadius,
                    center = canvasCenter + Offset(offsetX, offsetY)
                )
            }
        }
    }
}
