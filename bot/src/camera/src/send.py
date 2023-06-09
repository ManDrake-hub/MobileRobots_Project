import cv2
import asyncio
import websockets

async def capture_and_send(resolution=(1280, 720), fps=30):
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    capture.set(cv2.CAP_PROP_FPS, fps)

    # TODO: Receiver IP
    ws = await websockets.connect('ws://192.168.178.65:8000')  # Replace with the appropriate WebSocket server URL

    while True:
        ret, frame = capture.read()
        _, image_buffer = cv2.imencode('.jpg', frame)
        await ws.send(image_buffer.tobytes())

asyncio.get_event_loop().run_until_complete(capture_and_send())
