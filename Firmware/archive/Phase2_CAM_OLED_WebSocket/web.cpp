#include "web.h"
#include "config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

WebServer httpServer(HTTP_PORT);
WebSocketsServer webSocket(WEBSOCKET_PORT);

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-S3 CAM</title>

<style>
body {
    background: #101010;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
}

h2 {
    margin-bottom: 8px;
}

#status {
    margin-bottom: 15px;
}

#camera {
    width: 320px;
    max-width: 95vw;
    height: auto;
    border: 2px solid white;
    background: black;
}

button {
    margin: 10px 5px;
    padding: 10px 18px;
    font-size: 16px;
}
</style>
</head>

<body>

<h2>ESP32-S3 CAM</h2>

<div id="status">Connecting WebSocket...</div>

<img id="camera" alt="Camera stream">

<div>
    <button onclick="sendCommand('PING')">PING</button>
    <button onclick="sendCommand('BLINK')">BLINK</button>
</div>

<script>

const statusText = document.getElementById("status");
const camera = document.getElementById("camera");

let socket;

function connect()
{
    socket = new WebSocket("ws://" + window.location.hostname + ":81/");

    socket.binaryType = "blob";

    socket.onopen = function()
    {
        statusText.textContent = "WebSocket connected";
    };

    socket.onclose = function()
    {
        statusText.textContent = "WebSocket disconnected - retrying...";
        setTimeout(connect, 2000);
    };

    socket.onerror = function()
    {
        statusText.textContent = "WebSocket error";
    };

    socket.onmessage = function(event)
    {
        if (typeof event.data === "string")
        {
            statusText.textContent = event.data;
        }
        else
        {
            const oldURL = camera.src;
            const newURL = URL.createObjectURL(event.data);

            camera.onload = function()
            {
                if (oldURL.startsWith("blob:"))
                    URL.revokeObjectURL(oldURL);
            };

            camera.src = newURL;
        }
    };
}

function sendCommand(command)
{
    if (socket && socket.readyState === WebSocket.OPEN)
    {
        socket.send(command);
    }
}

connect();

</script>

</body>
</html>
)HTML";

static void webSocketEvent(
    uint8_t clientNum,
    WStype_t type,
    uint8_t *payload,
    size_t length
)
{
    switch (type)
    {
        case WStype_CONNECTED:
        {
            Serial.print("WebSocket client connected: ");
            Serial.println(clientNum);

            webSocket.sendTXT(
                clientNum,
                "WebSocket connected - waiting for camera frames"
            );
            break;
        }

        case WStype_DISCONNECTED:
        {
            Serial.print("WebSocket client disconnected: ");
            Serial.println(clientNum);
            break;
        }

        case WStype_TEXT:
        {
            Serial.print("WebSocket command: ");

            for (size_t i = 0; i < length; i++)
                Serial.write(payload[i]);

            Serial.println();

            if (length == 4 &&
                strncmp((const char *)payload, "PING", 4) == 0)
            {
                webSocket.sendTXT(clientNum, "PONG");
            }
            else if (length == 5 &&
                     strncmp((const char *)payload, "BLINK", 5) == 0)
            {
                webSocket.sendTXT(clientNum, "BLINK command received");
            }

            break;
        }

        default:
            break;
    }
}

bool wifiBegin()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("WIFI INITIALIZATION");
    Serial.println("================================");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print("Connecting to Wi-Fi");

    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED &&
           millis() - start < WIFI_CONNECT_TIMEOUT_MS)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Wi-Fi connection FAILED.");
        return false;
    }

    Serial.println("Wi-Fi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
}

void webBegin()
{
    httpServer.on("/", HTTP_GET, []()
    {
        httpServer.send_P(
            200,
            "text/html",
            INDEX_HTML
        );
    });

    httpServer.on("/status", HTTP_GET, []()
    {
        String json = "{\"wifi\":\"ok\",\"websocket\":81,\"camera\":\"websocket\"}";
        httpServer.send(
            200,
            "application/json",
            json
        );
    });

    httpServer.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    Serial.println("HTTP server started on port 80.");
    Serial.println("WebSocket server started on port 81.");
}

void webHandle()
{
    httpServer.handleClient();
    webSocket.loop();
}

void webSendCameraFrame(
    const uint8_t *data,
    size_t length
)
{
    if (webSocket.connectedClients() == 0)
        return;

    webSocket.broadcastBIN(
        data,
        length
    );
}
