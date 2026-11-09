"""Ключи и подписи в формате minisign (https://jedisct1.github.io/minisign/), Ed25519 через `cryptography`.

Решение Шифу 09.11.2026: подпись релиза — minisign. Файлы этого модуля совместимы с утилитой `minisign`:
подпись, сделанная здесь, проверяется `minisign -Vm release.yaml -p chuangong.pub`, и наоборот.
Поддержан только современный формат подписи `ED` (Ed25519 над BLAKE2b-512 от сообщения); старый `Ed`
(подпись всего сообщения) отвергается. Секретный ключ — только незашифрованный (`minisign -G -W`):
он живёт в секрете CI, пароль там некому вводить.

Один модуль на обе стороны: CI подписывает (`scripts/build/release.py sign`), агент проверяет.
"""

from __future__ import annotations

import base64
import hashlib
import os
from dataclasses import dataclass

_SIG_ALG_HASHED = b"ED"
_KEY_ALG = b"Ed"
_KDF_NONE = b"\x00\x00"
_CHK_ALG = b"B2"
_UNTRUSTED = "untrusted comment: "
_TRUSTED = "trusted comment: "


class SignatureError(ValueError):
    """Подпись не прошла проверку или файл ключа/подписи испорчен."""


@dataclass(frozen=True)
class PublicKey:
    keynum: bytes  # 8 байт, идентификатор ключа
    key: bytes  # 32 байта Ed25519

    @property
    def key_id(self) -> str:
        return self.keynum[::-1].hex().upper()


@dataclass(frozen=True)
class SecretKey:
    keynum: bytes
    seed: bytes  # 32 байта Ed25519 seed
    public: bytes  # 32 байта


def _ed25519():
    # Ленивый импорт: наблюдателю workflow (status) криптография не нужна.
    from cryptography.hazmat.primitives.asymmetric import ed25519

    return ed25519


def _payload_line(text: str, what: str) -> bytes:
    lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
    if len(lines) < 2 or not lines[0].startswith(_UNTRUSTED):
        raise SignatureError(f"{what}: ожидалась строка «untrusted comment:» и строка base64")
    try:
        return base64.b64decode(lines[1], validate=True)
    except ValueError as exc:
        raise SignatureError(f"{what}: битый base64") from exc


def generate_keypair() -> tuple[str, str]:
    """(текст секретного ключа, текст публичного ключа) — для тестов и первого ключа CI."""
    ed = _ed25519()
    priv = ed.Ed25519PrivateKey.generate()
    seed = priv.private_bytes_raw()
    pub = priv.public_key().public_bytes_raw()
    keynum = os.urandom(8)
    sk = seed + pub
    chk = hashlib.blake2b(_KEY_ALG + keynum + sk, digest_size=32).digest()
    blob = _KEY_ALG + _KDF_NONE + _CHK_ALG + bytes(32) + bytes(8) + bytes(8) + keynum + sk + chk
    secret = f"{_UNTRUSTED}minisign secret key (unencrypted)\n{base64.b64encode(blob).decode()}\n"
    pk = PublicKey(keynum, pub)
    public = f"{_UNTRUSTED}minisign public key {pk.key_id}\n{base64.b64encode(_KEY_ALG + keynum + pub).decode()}\n"
    return secret, public


def load_public_key(text: str) -> PublicKey:
    raw = _payload_line(text, "публичный ключ")
    if len(raw) != 42 or raw[:2] != _KEY_ALG:
        raise SignatureError("публичный ключ: не Ed25519-ключ minisign")
    return PublicKey(keynum=raw[2:10], key=raw[10:42])


def load_secret_key(text: str) -> SecretKey:
    raw = _payload_line(text, "секретный ключ")
    if len(raw) != 158 or raw[:2] != _KEY_ALG or raw[4:6] != _CHK_ALG:
        raise SignatureError("секретный ключ: не Ed25519-ключ minisign")
    if raw[2:4] != _KDF_NONE:
        raise SignatureError("секретный ключ зашифрован паролем: для CI нужен незашифрованный (minisign -G -W)")
    keynum, sk, chk = raw[54:62], raw[62:126], raw[126:158]
    if hashlib.blake2b(_KEY_ALG + keynum + sk, digest_size=32).digest() != chk:
        raise SignatureError("секретный ключ: не сошлась контрольная сумма")
    return SecretKey(keynum=keynum, seed=sk[:32], public=sk[32:])


def sign(message: bytes, secret: SecretKey, trusted_comment: str, untrusted_comment: str = "signature") -> str:
    """Текст файла `.minisig` для сообщения."""
    if "\n" in trusted_comment or "\r" in trusted_comment:
        raise ValueError("trusted comment — одна строка")
    priv = _ed25519().Ed25519PrivateKey.from_private_bytes(secret.seed)
    sig = priv.sign(hashlib.blake2b(message, digest_size=64).digest())
    global_sig = priv.sign(sig + trusted_comment.encode())
    return (
        f"{_UNTRUSTED}{untrusted_comment}\n"
        f"{base64.b64encode(_SIG_ALG_HASHED + secret.keynum + sig).decode()}\n"
        f"{_TRUSTED}{trusted_comment}\n"
        f"{base64.b64encode(global_sig).decode()}\n"
    )


def verify(message: bytes, signature_text: str, public: PublicKey) -> str:
    """Проверяет подпись и trusted comment; возвращает trusted comment. Любое расхождение — SignatureError."""
    lines = [ln.rstrip("\r") for ln in signature_text.strip().splitlines()]
    if len(lines) != 4 or not lines[0].startswith(_UNTRUSTED) or not lines[2].startswith(_TRUSTED):
        raise SignatureError("подпись: ожидалось 4 строки формата minisign")
    try:
        raw = base64.b64decode(lines[1], validate=True)
        global_sig = base64.b64decode(lines[3], validate=True)
    except ValueError as exc:
        raise SignatureError("подпись: битый base64") from exc
    if len(raw) != 74 or len(global_sig) != 64:
        raise SignatureError("подпись: неверная длина")
    if raw[:2] != _SIG_ALG_HASHED:
        raise SignatureError(f"подпись: алгоритм {raw[:2]!r} не поддержан (нужен ED, prehashed)")
    if raw[2:10] != public.keynum:
        raise SignatureError("подпись сделана другим ключом (keynum не совпал)")
    trusted = lines[2][len(_TRUSTED) :]
    ed = _ed25519()
    pub = ed.Ed25519PublicKey.from_public_bytes(public.key)
    from cryptography.exceptions import InvalidSignature

    try:
        pub.verify(raw[10:], hashlib.blake2b(message, digest_size=64).digest())
        pub.verify(global_sig, raw[10:] + trusted.encode())
    except InvalidSignature as exc:
        raise SignatureError("подпись не сошлась: сообщение или trusted comment изменены") from exc
    return trusted
