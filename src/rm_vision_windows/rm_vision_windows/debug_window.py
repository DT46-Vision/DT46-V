"""Control the Python vision task through the same debug transport as native DT46."""

import argparse
import json
import os
from pathlib import Path
import queue
import secrets
import socket
import threading

from .transport import Connection


SESSION = Path(__file__).resolve().parents[3] / ".local/virtual_camera/session.json"


class DebugControl:
    def __init__(self, session: Path = SESSION):
        self.session, self.key = session, secrets.token_hex(32)
        self.commands = queue.Queue()
        self.state = {}
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(4)
        self.listener.settimeout(0.05)
        session.parent.mkdir(parents=True, exist_ok=True)
        session.write_text(json.dumps({"protocol": 1, "port": self.listener.getsockname()[1],
                                       "key": self.key, "pid": os.getpid()}), encoding="utf-8")
        self.thread = threading.Thread(target=self.serve, name="dt46-debug", daemon=True)
        self.thread.start()

    def publish(self, state: dict) -> None:
        with self.lock:
            self.state = state

    def serve(self) -> None:
        while not self.stopped.is_set():
            try:
                stream, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            connection = Connection(stream)
            try:
                request, _ = connection.receive()
                if not secrets.compare_digest(request.get("key", ""), self.key):
                    raise ValueError("Invalid session key")
                operation = request.get("op")
                if operation == "inspect":
                    with self.lock:
                        connection.send({"status": self.state})
                elif operation in ("show", "close", "save", "configure", "reset", "load_parameters", "save_parameters"):
                    self.commands.put(request)
                    connection.send({"requested": True})
                else:
                    raise ValueError("Unknown debug operation")
            except (ValueError, OSError, ConnectionError) as error:
                try:
                    connection.send({"error": str(error)})
                except OSError:
                    pass
            finally:
                connection.close()

    def close(self) -> None:
        self.stopped.set()
        self.listener.close()
        self.thread.join(2.1)
        if self.session.exists() and json.loads(self.session.read_text(encoding="utf-8")).get("key") == self.key:
            self.session.unlink()


def command(session: Path, operation: str, **values) -> dict:
    description = json.loads(session.read_text(encoding="utf-8"))
    connection = Connection.connect(description["port"])
    try:
        return connection.request({"key": description["key"], "op": operation, **values})[0]
    finally:
        connection.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=SESSION)
    parser.add_argument("--action", choices=("show", "close", "save", "inspect", "reset", "configure", "load_parameters", "save_parameters"), default="show")
    parser.add_argument("--parameters", type=Path)
    args = parser.parse_args()
    values = {}
    if args.action == "configure":
        if args.parameters is None:
            parser.error("configure requires --parameters")
        values["parameters"] = json.loads(args.parameters.read_text(encoding="utf-8"))
    elif args.action in ("load_parameters", "save_parameters"):
        if args.parameters is None:
            parser.error("load/save requires --parameters")
        values["path"] = str(args.parameters.resolve())
    print(json.dumps(command(args.session, args.action, **values)))


if __name__ == "__main__":
    main()
