import asyncio
import threading
import tkinter as tk
from queue import Queue, Empty
import websockets

# Configuration
SERVER_URI = "ws://192.168.3.111:8765"  # adjust to your server address
REQUEST_INTERVAL = 0.5  # seconds between requests

class WSClient:
    def __init__(self, uri, queue):
        self.uri = uri
        self.queue = queue
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.start_loop, daemon=True)

    def start(self):
        self.thread.start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    async def run(self):
        try:
            async with websockets.connect(self.uri) as ws:
                while True:
                    # send a request to the server
                    await ws.send("get_info")
                    # receive the server's response
                    message = await ws.recv()
                    self.queue.put(message)
                    # wait before next request
                    await asyncio.sleep(REQUEST_INTERVAL)
        except Exception as e:
            self.queue.put(f"[Error] {e}")

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Real-Time WebSocket Client")
        self.geometry("400x300")

        self.text = tk.Text(self, state='disabled', wrap='word')
        self.text.pack(fill='both', expand=True, padx=10, pady=10)

        self.status = tk.Label(self, text="Disconnected", anchor='w')
        self.status.pack(fill='x', padx=10, pady=(0,10))

        # message queue from websocket thread
        self.queue = Queue()

        # start websocket client
        self.client = WSClient(SERVER_URI, self.queue)
        self.client.start()
        self.status.config(text="Connecting...")

        # start polling for messages
        self.after(100, self.poll_queue)

    def poll_queue(self):
        try:
            while True:
                msg = self.queue.get_nowait()
                self.display_message(msg)
        except Empty:
            pass
        # keep polling
        self.after(100, self.poll_queue)

    def display_message(self, msg):
        self.text.config(state='normal')
        self.text.insert('end', msg + "\n")
        self.text.see('end')
        self.text.config(state='disabled')
        # update status on first message
        if self.status['text'] != 'Connected':
            self.status.config(text='Connected')

if __name__ == '__main__':
    app = App()
    app.mainloop()