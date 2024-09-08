#!/usr/bin/env python

import asyncio
from websockets.sync.client import connect
import PIL.Image as Image
import io
from matplotlib import pyplot as plt
from matplotlib import image as mpimg
from time import time
from tqdm import trange


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

        plt.imshow(mpimg.imread(io.BytesIO(message)))
        plt.show()


def main():

    pil_im = Image.open("data/arbor-logo.png")
    b = io.BytesIO()
    pil_im.save(b, "png")
    im_bytes = b.getvalue()

    # print(im_bytes)
    hello(im_bytes)
