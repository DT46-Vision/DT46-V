"""DT46V001 transport shared by the Windows camera and debug adapters."""

import json
import socket
import struct


class ConnectionClosed(ConnectionError):
    pass


class Connection:
    def __init__(self, stream: socket.socket):
        self.stream = stream
        stream.settimeout(2)
        stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    @classmethod
    def connect(cls, port: int):
        return cls(socket.create_connection(("127.0.0.1", port), timeout=2))

    def close(self) -> None:
        self.stream.close()

    def send(self, metadata: dict, pixels: bytes = b"") -> None:
        data = json.dumps(metadata, allow_nan=False, separators=(",", ":")).encode()
        if not 0 < len(data) <= 1024**2 or len(pixels) > 64 * 1024**2:
            raise ValueError("Transport packet exceeds size limit")
        self.stream.sendall(struct.pack("<8sII", b"DT46V001", len(data), len(pixels)) + data + pixels)

    def read(self, length: int) -> bytes:
        data = bytearray()
        while len(data) < length:
            try:
                part = self.stream.recv(length - len(data))
            except OSError as error:
                if isinstance(error, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)) or getattr(error, "winerror", None) == 10058:
                    raise ConnectionClosed(str(error)) from error
                raise
            if not part:
                raise ConnectionClosed("Peer closed the connection")
            data.extend(part)
        return bytes(data)

    def receive(self) -> tuple[dict, bytes]:
        magic, json_size, binary_size = struct.unpack("<8sII", self.read(16))
        if magic != b"DT46V001" or not 0 < json_size <= 1024**2 or binary_size > 64 * 1024**2:
            raise ValueError("Invalid camera/debug transport packet")
        return json.loads(self.read(json_size)), self.read(binary_size)

    def request(self, metadata: dict) -> tuple[dict, bytes]:
        self.send(metadata)
        reply, pixels = self.receive()
        if "error" in reply:
            raise ValueError(reply["error"])
        return reply, pixels
