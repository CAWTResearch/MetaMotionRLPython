import asyncio
import websockets

async def hello():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        name = input("Enter your name: ")
        await websocket.send(name)
        print(f"Sent name: {name}")

        greeting = await websocket.recv()
        print(f"Received greeting: {greeting}")

if __name__ == "__main__":
    asyncio.run(hello())
    