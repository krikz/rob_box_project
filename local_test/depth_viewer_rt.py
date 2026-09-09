#!/usr/bin/env python3
"""Real-time depth + color viewer via Zenoh/ROS2 compressed topics."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading, time, io, struct, numpy as np
from PIL import Image as PILImage, ImageTk
import tkinter as tk

DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw/compressedDepth'
COLOR_TOPIC = '/camera/camera/color/image_raw/compressed'

class Viewer(Node):
    def __init__(self):
        super().__init__('depth_viewer_rt')
        qos = QoSProfile(depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CompressedImage, DEPTH_TOPIC, self._depth_cb, qos)
        self.create_subscription(CompressedImage, COLOR_TOPIC, self._color_cb, qos)
        self.latest_depth = None
        self.latest_color = None
        self.lock = threading.Lock()
        self.fps_depth = 0
        self.fps_color = 0
        self._depth_times = []
        self._color_times = []

    def _depth_cb(self, msg):
        data = bytes(msg.data)
        png_start = data.find(b'\x89PNG')
        if png_start < 0:
            return
        try:
            img = PILImage.open(io.BytesIO(data[png_start:]))
            arr = np.array(img).astype(np.float32)
            valid = arr[arr > 0]
            if not len(valid):
                return
            vmin, vmax = valid.min(), valid.max()
            # Jet colormap
            norm = np.clip((arr - vmin) / (vmax - vmin + 1e-6), 0, 1)
            r = np.clip(1.5 - abs(norm*4 - 3), 0, 1)
            g = np.clip(1.5 - abs(norm*4 - 2), 0, 1)
            b = np.clip(1.5 - abs(norm*4 - 1), 0, 1)
            rgb = (np.stack([r,g,b], axis=2) * 255).astype(np.uint8)
            rgb[arr <= 0] = 0
            pil = PILImage.fromarray(rgb).resize((640, 360))
            t = time.time()
            self._depth_times = [x for x in self._depth_times if t - x < 2.0] + [t]
            self.fps_depth = len(self._depth_times) / 2.0
            with self.lock:
                self.latest_depth = pil
        except Exception as e:
            pass

    def _color_cb(self, msg):
        try:
            img = PILImage.open(io.BytesIO(bytes(msg.data))).resize((640, 360))
            t = time.time()
            self._color_times = [x for x in self._color_times if t - x < 2.0] + [t]
            self.fps_color = len(self._color_times) / 2.0
            with self.lock:
                self.latest_color = img
        except Exception as e:
            pass


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = Viewer()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title("Rob Box — Depth + Color Viewer")
    root.configure(bg='black')

    lbl_color = tk.Label(root, bg='black')
    lbl_color.grid(row=0, column=0, padx=5, pady=5)
    lbl_depth = tk.Label(root, bg='black')
    lbl_depth.grid(row=0, column=1, padx=5, pady=5)

    info_color = tk.Label(root, text="Color: waiting...", bg='black', fg='white')
    info_color.grid(row=1, column=0)
    info_depth = tk.Label(root, text="Depth: waiting...", bg='black', fg='white')
    info_depth.grid(row=1, column=1)

    placeholder = PILImage.new('RGB', (640, 360), (30, 30, 30))
    ph_tk = ImageTk.PhotoImage(placeholder)
    lbl_color.config(image=ph_tk); lbl_color.image = ph_tk
    lbl_depth.config(image=ph_tk); lbl_depth.image = ph_tk

    def update():
        with node.lock:
            color = node.latest_color
            depth = node.latest_depth
            fps_c = node.fps_color
            fps_d = node.fps_depth

        if color:
            tk_img = ImageTk.PhotoImage(color)
            lbl_color.config(image=tk_img)
            lbl_color.image = tk_img
            info_color.config(text=f"Color: {fps_c:.1f} fps")

        if depth:
            tk_img = ImageTk.PhotoImage(depth)
            lbl_depth.config(image=tk_img)
            lbl_depth.image = tk_img
            info_depth.config(text=f"Depth (jet): {fps_d:.1f} fps")

        root.after(100, update)

    root.after(100, update)
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
