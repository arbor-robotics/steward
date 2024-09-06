#!/usr/bin/env python

import asyncio
from websockets.asyncio.server import serve

import importlib.util
import sys

from sim_bridge.protos.std_msgs import helloworld_pb2, helloworld_pb2_grpc

import grpc
from concurrent import futures


class Greeter(helloworld_pb2_grpc.GreeterServicer):
    def SayHello(self, request, context):
        return helloworld_pb2.HelloReply(message="Hello, %s!" % request.name)


def serve():
    port = "50051"
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    helloworld_pb2_grpc.add_GreeterServicer_to_server(Greeter(), server)
    server.add_insecure_port("[::]:" + port)
    server.start()
    print("Server started, listening on " + port)
    server.wait_for_termination()


def main():
    # logging.basicConfig()
    serve()
