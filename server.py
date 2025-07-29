import asyncio, websockets

async def server(websocket):
    print("Client connected")
    try:
        async for msg in websocket:
            # e.g. respond to “get_info”
            if msg == "get_info" or msg == '"get_info"':
                info = get_realtime_info()       # your custom function
                await websocket.send(info)
            else:
                # fallback / logging
                await websocket.send(f"Unknown command: {msg}")
    except websockets.ConnectionClosed:
        print("Client disconnected")


def get_realtime_info():
    # gather whatever you need here; stub:
    print("Gathering realtime info...")
    return "0"

async def start():
    async with websockets.serve(server, "0.0.0.0", 8765):
        print("Server listening on 0.0.0.0:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(start())
