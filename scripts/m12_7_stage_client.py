#!/usr/bin/env python3
import os
import sys
import time
import socket
import json

SOCKET_PATH = "/tmp/m12_7_daemon.sock"

def send_command(cmd_str, countdown=5):
    if not os.path.exists(SOCKET_PATH):
        print(f"Error: Daemon socket {SOCKET_PATH} does not exist.", flush=True)
        sys.exit(1)

    if countdown > 0:
        print(f"\n==========================================", flush=True)
        print(f"STAGE RECORDING COUNTDOWN ({countdown}s):", flush=True)
        for c in range(countdown, 0, -1):
            print(f"  >>> {c} ...", flush=True)
            time.sleep(1.0)
        print(f"  >>> RECORDING — MOVE NOW! <<<", flush=True)
        print(f"==========================================\n", flush=True)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(SOCKET_PATH)
    s.sendall(cmd_str.encode('utf-8'))

    # Receive response
    data = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        data += chunk
    s.close()
    
    if data:
        try:
            res = json.loads(data.decode('utf-8'))
            print(json.dumps(res, indent=2), flush=True)
        except Exception:
            print(data.decode('utf-8'), flush=True)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: m12_7_stage_client.py <CMD> [--countdown SECONDS]")
        sys.exit(1)
    
    countdown = 5
    args = sys.argv[1:]
    if "--countdown" in args:
        idx = args.index("--countdown")
        countdown = int(args[idx + 1])
        args = args[:idx] + args[idx + 2:]

    send_command(" ".join(args), countdown=countdown)
