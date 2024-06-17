
import signal
import sys
import time
import websockets
import asyncio
import threading
import numpy as np
import cv2
import json

# width = 1280
# height = 640
# src_points = np.float32([[0, 0], [640, 0], [640, 480]])
# dst_points = np.float32([[0, 0], [width, 0], [width, height]])
# frameH = 0
# frameW = 0
stopFlag = False
ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


class ARucoWorker (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.data = {}
        self.lastData = {}

    # Simulate GPS data
    def run(self):
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Camera H, W:", height, width)
        frameCount = 0
        showIm = True
        while not stopFlag:
            ret, frame = cap.read()
            # frame = cv2.resize(frame, (1200, 800))

            # Get the affine transform matrix
            # M = cv2.getAffineTransform(src_points, dst_points)
            # Apply the affine transform to the frame
            # m_frame = cv2.warpAffine(frame, M, (width, height))
            if not ret:
                break
            if frame is None:
                continue
            frameCount += 1
            height, width, _ = frame.shape
            # print("Camera H, W:", height, width)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Gray", gray)
            result = []
            # result.append({"frameH": frameH, "frameW": frameW})
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                ids = ids.flatten()
                for id, corner in zip(ids, corners):
                    corners = corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # Find center
                    cx = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cy = int((topLeft[1] + bottomRight[1]) / 2.0)
                    result.append({"id": int(id), "cx": width - cx, "cy": cy})
            if len(result) > 0:
                self.data = result
            frame = cv2.flip(frame, 1)
            cv2.imshow("Frame", frame)

            key = cv2.waitKey(10)
            if key == 27:
                break

    def get(self):
        if self.lastData is not self.data:
            self.lastData = self.data
            return self.data


class MSGWorker (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.connected = set()
        self.frequency = 0.1     # 10 data / s

    def run(self):
        while not stopFlag:
            data = arWorker.get()
            if data:
                self.sendData(json.dumps(data))
            time.sleep(self.frequency)

    async def handler(self, websocket, path):
        print("New client connected!")
        self.connected.add(websocket)
        try:
            async for data in websocket:
                print("Received:", data)
        except websockets.exceptions.ConnectionClosed:
            print("disconnected")
        except:
            print("error")
        # finally:1
        #   self.connected.remove(websocket)

    def sendData(self, data):
        for websocket in self.connected.copy():
            coro = websocket.send(data)
            future = asyncio.run_coroutine_threadsafe(coro, loop)


if __name__ == "__main__":
    print('Game server')
    arWorker = ARucoWorker()
    msgWorker = MSGWorker()

    try:
        arWorker.start()
        msgWorker.start()

        ws_server = websockets.serve(msgWorker.handler, 'localhost', 3030)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(ws_server)
        loop.run_forever()
    except KeyboardInterrupt:
        stopFlag = True
        # TODO: close ws server and loop correctly
        print("Exiting program...")
