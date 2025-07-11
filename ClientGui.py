import asyncio
import threading
import tkinter as tk
from queue import Queue, Empty
import websockets
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Configuration
SERVER_URI = "ws://192.168.3.111:8765"
REQUEST_INTERVAL = 0.5  # seconds between requests

# Movement categories and associated image files
CATEGORIES = ["sitting down", "folding clothes", "sweeping", "walking", "moving boxes", "running bicycle"]

PredictionsMap = {0:"sitting down", 1:"folding clothes", 2:"sweeping", 3:"walking", 4:"moving boxes", 5:"running bicycle"}

IMAGE_PATHS = {cat: f"images/{cat.replace(' ', '_')}.png" for cat in CATEGORIES}

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
        while True:
            try:
                async with websockets.connect(self.uri) as ws:
                    while True:
                        await ws.send("get_info")
                        msg = await ws.recv()
                        self.queue.put(msg)
                        await asyncio.sleep(REQUEST_INTERVAL)
            except websockets.ConnectionClosedOK:
                self.queue.put("[Info] Reconnecting...")
                await asyncio.sleep(1)
            except Exception as e:
                self.queue.put(f"[Error] {e}")
                break

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Real-Time Activity Predictions")
        self.geometry("1000x1000")

        # Frame for image display
        self.img_label = tk.Label(self)
        self.img_label.pack(pady=5)

        # Matplotlib figure for bar chart
        self.fig, self.ax = plt.subplots(figsize=(5,2))
        self.counts = {cat: 0 for cat in CATEGORIES}
        self.bar_chart = FigureCanvasTkAgg(self.fig, master=self)
        self.bar_chart.get_tk_widget().pack(fill='both', expand=True)

        # Status label
        self.status = tk.Label(self, text="Connecting...", anchor='w')
        self.status.pack(fill='x', padx=10, pady=(0,5))

        # Queue and WS client
        self.queue = Queue()
        self.client = WSClient(SERVER_URI, self.queue)
        self.client.start()

        # Start polling
        self.after(100, self.poll_queue)

    def poll_queue(self):
        try:
            while True:
                msg = self.queue.get_nowait()
                self.handle_message(msg)
        except Empty:
            pass
        self.after(100, self.poll_queue)

    def handle_message(self, msg):

        # Update status
        if self.status['text'] != 'Connected':
            self.status.config(text='Connected')
        
        msg = PredictionsMap[msg]

        # If msg is a known category, update chart and image
        if msg in CATEGORIES:
            self.counts[msg] += 1
            self.update_chart()
            self.update_image(msg)

    def update_chart(self):
        self.ax.clear()
        cats = list(self.counts.keys())
        vals = [self.counts[c] for c in cats]
        self.ax.bar(cats, vals)
        self.ax.set_ylabel('Count')
        self.ax.set_xticklabels(cats, rotation=45, ha='right')
        self.fig.tight_layout()
        self.bar_chart.draw()

    def update_image(self, category):
        path = IMAGE_PATHS.get(category)
        print(f"Loading image for {category} from {path}")
        img = Image.open(path)
        img = img.resize((400, 400))
        tk_img = ImageTk.PhotoImage(img)
        self.img_label.config(image=tk_img)
        self.img_label.image = tk_img
        
        

if __name__ == '__main__':
    app = App()
    app.mainloop()
