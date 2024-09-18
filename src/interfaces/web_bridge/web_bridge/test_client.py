#!/usr/bin/env python

import asyncio
from websockets.sync.client import connect
import PIL.Image as Image
import io
from matplotlib import pyplot as plt
from matplotlib import image as mpimg
from time import time
from tqdm import trange


class MessageType:
    IMAGE = 0x00
    SUBSCRIBE = 0x01
    TELEOP = 0x02


def formSubscribeOp(message_type: MessageType, topic_id=0):
    # First byte is "subscribe to", second byte is type
    # third byte is in case there is more than one source
    # for this type (e.g. multiple cameras)
    return bytes([MessageType.SUBSCRIBE, message_type, topic_id])


def hello(data="Hello, world"):
    with connect("ws://localhost:8765") as websocket:

        avg_hz = 0
        for i in trange(100):
            start = time()
            websocket.send(data)
            message = websocket.recv()
            # print(f"Received: {message}")
            delay = time() - start
            print(f"Took {delay} secs ({1/delay} Hz)")
            avg_hz += 1 / delay

        print(f"Avg({avg_hz/100} Hz)")

        plt.imshow(mpimg.imread(io.BytesIO(message[1:])))
        plt.show()


def requestTeleop():
    with connect("ws://localhost:8765") as websocket:
        sub_request = formSubscribeOp(MessageType.TELEOP)
        websocket.send(sub_request)

        while True:
            message = websocket.recv()
            print(message)

        # print(f"Avg({avg_hz/100} Hz)")

        # plt.imshow(mpimg.imread(io.BytesIO(message[1:])))
        # plt.show()


def main():

    pil_im = Image.open("data/arbor-logo.png")
    b = io.BytesIO()
    pil_im.save(b, "png")
    im_bytes = b.getvalue()

    message_type = MessageType.IMAGE

    # print(im_bytes)
    # hello(bytes([message_type]) + im_bytes)
    requestTeleop()
