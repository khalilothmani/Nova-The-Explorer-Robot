"""
ESP32-CAM AI-Thinker Video Streaming Example (MicroPython)

Instructions:

1. Flash MicroPython firmware onto your ESP32-CAM:
   - Download the latest MicroPython firmware for ESP32 from: https://micropython.org/download/esp32/
   - Use esptool.py to flash:
       esptool.py --chip esp32 --port COM3 erase_flash
       esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 esp32-xxxxxx.bin

2. Connect your ESP32-CAM AI-Thinker module:
   - Use a USB-to-Serial adapter (FTDI) with 5V logic.
   - GPIO0 must be LOW while flashing.
   - Connect GND, 5V, TX, RX.

3. Use Thonny, uPyCraft, or ampy to upload this code to the ESP32-CAM.

4. Replace 'YOUR_SSID' and 'YOUR_PASSWORD' with your Wi-Fi credentials.

5. After booting, the ESP32-CAM will print its IP address. Open it in a browser to see the video stream.
"""

import network
import socket
import time
from machine import Pin
import esp32
import camera

# Wi-Fi credentials
SSID = 'YOUR_SSID'
PASSWORD = 'YOUR_PASSWORD'

# Initialize the camera
camera.init(0, format=camera.JPEG)
camera.framesize(camera.FRAME_VGA)  # 640x480
camera.quality(10)  # JPEG quality 10 (lower is better compression)

# Connect to Wi-Fi
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(SSID, PASSWORD)
print("Connecting to Wi-Fi...")

while not wifi.isconnected():
    time.sleep(0.5)
print("Connected! IP:", wifi.ifconfig()[0])

# Set up socket server
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print("Listening on", addr)

def serve_stream(conn):
    """Serve a single JPEG image"""
    buf = camera.capture()
    conn.send(b'HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\n\r\n' + buf)

while True:
    cl, addr = s.accept()
    print('Client connected from', addr)
    serve_stream(cl)
    cl.close()
