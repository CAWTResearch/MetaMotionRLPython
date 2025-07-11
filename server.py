import asyncio, websockets

async def hello(websocket):
    print("Client connected")
    try:
        async for msg in websocket:
            # e.g. respond to “get_info”
            if msg == "get_info":
                info = get_realtime_info()       # your custom function
                await websocket.send(info)
            else:
                # fallback / logging
                await websocket.send(f"Unknown command: {msg}")
    except websockets.ConnectionClosed:
        print("Client disconnected")


def get_realtime_info():
    # gather whatever you need here; stub:
    return "walking"

async def main():
    async with websockets.serve(hello, "0.0.0.0", 8765):
        print("Server listening on 0.0.0.0:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
