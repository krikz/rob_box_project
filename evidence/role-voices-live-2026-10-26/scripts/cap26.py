"""cap26.py : пишет каждое сообщение /audio/speech_audio (то, что получает speaker_id_node) в /vp26/cap/<мс>_<байт>.raw.
Запуск: одноразовый контейнер (ros1.sh), НЕ внутри сервисного. Как cap.py 23.10."""
import time, os, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from audio_common_msgs.msg import AudioData

os.makedirs("/vp26/cap", exist_ok=True)


class Cap(Node):
    def __init__(self):
        super().__init__("vp_cap_3992")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=20)
        self.create_subscription(AudioData, "/audio/speech_audio", self.cb, q)

    def cb(self, msg):
        t = time.time()
        b = bytes(msg.data)
        fn = "/vp26/cap/%d_%d.raw" % (int(t * 1000), len(b))
        with open(fn, "wb") as f:
            f.write(b)
        print("cap", fn, flush=True)


rclpy.init()
n = Cap()
print("ready", flush=True)
rclpy.spin(n)
