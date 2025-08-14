import asyncio, websockets

async def server(websocket):
    print("Client connected")
    try:
        async for msg in websocket:
            # e.g. respond to “get_info”
            print(f"Received message: {msg}")
            if msg == "get_info" or msg == '"get_info"':
                info = get_realtime_info()       # your custom function
                await websocket.send(info)

    except websockets.ConnectionClosed:
        print("Client disconnected")

def mode(message):
    modes = {"Standby": 0,
             "Measurement": 1, "Calibration": 2, "Diagnostics": 3}
    if not message in modes:
        return 0

    return modes[message]




def get_realtime_info():
    # gather whatever you need here; stub:
    print("Gathering realtime info...")
    return "1"

async def start():
    async with websockets.serve(server, "0.0.0.0", 8765):
        print("Server listening on 0.0.0.0:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(start())
