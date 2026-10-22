"""Фейковый RequestAuthenticator: подпись -- пара полей; FAKE_NO_SIGN=1 -- секрета нет (см. rclpy/__init__.py)."""

import os


class RequestAuthenticator:
    def __init__(self, sender):
        self.sender = sender
        self.can_sign = os.environ.get("FAKE_NO_SIGN") != "1"

    @classmethod
    def from_env(cls, sender="harness"):
        return cls(sender)

    def sign(self, request):
        request["sender"] = self.sender
        request["signature"] = "fake-signature"
        return request
