#!/usr/bin/env python

import asyncio
from websockets.sync.client import connect

import cv2
import numpy as np
from matplotlib import pyplot as plt


def hello():

    image: np.ndarray = cv2.imread("data/arbor-logo.png")

    print(image.shape)
    original_shape = image.shape
    image = image.tobytes()

    with connect("ws://localhost:8787") as websocket:
        websocket.send(image)
        message = websocket.recv()
        # print(f"Received: {message}")

        received_image = np.frombuffer(message, dtype=np.uint8).reshape(original_shape)
        print(received_image)
        plt.imshow(received_image)
        plt.show()


def main():
    hello()
