import cv2
import asyncio
import websockets
import numpy as np

async def receive_and_display(websocket, path):
    while True:
        image_buffer = await websocket.recv()
        nparr = np.frombuffer(image_buffer, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow('Received Image', frame)

        if cv2.waitKey(1) == 27:  # Exit when Esc key is pressed
            break

    cv2.destroyAllWindows()

# TODO: Receiver IP
start_server = websockets.serve(receive_and_display, '192.168.178.65', 8000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
