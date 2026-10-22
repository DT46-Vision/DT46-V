"""Open the debug window of the existing DT46-V non-ROS task."""

import argparse
import json
import os
from pathlib import Path
import secrets
import threading
from multiprocessing.connection import Client, Listener


SESSION = Path(__file__).resolve().parents[1]/".cache/virtual_camera/session.json"


class DebugControl:
    def __init__(self):
        self.requested = threading.Event()
        self.key = secrets.token_bytes(32)
        self.listener = Listener(("127.0.0.1", 0), authkey=self.key)
        SESSION.parent.mkdir(parents=True, exist_ok=True)
        SESSION.write_text(json.dumps({"pid": os.getpid(), "port": self.listener.address[1],
                                       "key": self.key.hex()}), encoding="utf-8")
        threading.Thread(target=self.serve, name="dt46-debug-command", daemon=True).start()

    def serve(self) -> None:
        while True:
            try:
                with self.listener.accept() as connection:
                    if connection.recv() == "show":
                        self.requested.set()
                        connection.send("opened")
            except (OSError, EOFError):
                return

    def close(self) -> None:
        self.listener.close()
        if SESSION.exists() and json.loads(SESSION.read_text(encoding="utf-8"))["pid"] == os.getpid():
            SESSION.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=SESSION)
    args = parser.parse_args()
    description = json.loads(args.session.read_text(encoding="utf-8"))
    with Client(("127.0.0.1", description["port"]), authkey=bytes.fromhex(description["key"])) as connection:
        connection.send("show")
        connection.recv()


if __name__ == "__main__":
    main()
