"""Фейковые rob_box_dialog_msgs (issue #3989): снимок сессии ``/dialog/session`` и реплика ``/voice/stt/utterance``.

Поля -- как в ``src/rob_box_dialog_msgs/msg/{SessionState,SceneFrame,Utterance}.msg`` (только нужные харнессу).
Снимок строится из ``$FAKE_ROBOT_STATE/session_state.json`` (``{"agent": ..., "scenes": [{"kind", "set_by",
"ttl_until"}]}``), см. ``rclpy/__init__.py``.
"""


class Time:
    def __init__(self, t=0.0):
        self.sec = int(t)
        self.nanosec = int((t - int(t)) * 1e9)


class SceneFrame:
    def __init__(self, kind="", set_by="", ttl_until=0.0):
        self.kind, self.set_by, self.ttl_until, self.payload_json = kind, set_by, Time(ttl_until), "{}"


class SessionState:
    def __init__(self, agent="", scenes=()):
        self.agent = agent
        self.scenes = list(scenes)

    @classmethod
    def from_dict(cls, data):
        return cls(data.get("agent", ""), [SceneFrame(s.get("kind", ""), s.get("set_by", ""),
                                                      float(s.get("ttl_until") or 0.0))
                                           for s in data.get("scenes", [])])


class Utterance:
    SOURCE_RESPEAKER = "respeaker"
    SOURCE_TELEGRAM = "telegram"
    SOURCE_OPERATOR = "operator"

    def __init__(self):
        self.utterance_id = ""
        self.text = ""
        self.source = ""
        self.chat_id = 0
        self.speaker_tag = ""
        self.provider = ""
