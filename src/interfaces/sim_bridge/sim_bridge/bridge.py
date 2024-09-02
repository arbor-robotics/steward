#!/usr/bin/env python

import asyncio
from websockets.asyncio.server import serve


async def echo(websocket):
    async for message in websocket:
        # print("YEET")
        await websocket.send(message)


async def main():
    async with serve(echo, "localhost", 8787, max_size=2**21):
        await asyncio.get_running_loop().create_future()  # run forever


asyncio.run(main())
