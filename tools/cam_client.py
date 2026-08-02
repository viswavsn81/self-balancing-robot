#!/usr/bin/env python3
"""Robot camera viewer + wireless console client.

Connects to the ESP32 bridge (default robot-cam.local):
  - shows the MJPEG camera stream (OpenCV window; 'q' closes)
  - opens the WebSocket command channel; whatever you type is sent as a
    framed console command ($payload*XOR) and robot replies/telemetry
    stream to stdout. 'x' is the e-stop, exactly like the USB console.

Usage:
    python3 tools/cam_client.py [--host robot-cam.local] [--no-video]
    python3 tools/cam_client.py --send "?"        # one-shot command
"""

import argparse
import sys
import threading
import time

import websocket  # pip install websocket-client


def frame(cmd: str) -> str:
    cs = 0
    for ch in cmd:
        cs ^= ord(ch)
    return f"${cmd}*{cs:02X}"


def video_loop(url: str, stop: threading.Event) -> None:
    try:
        import cv2
    except ImportError:
        print(f"(no OpenCV — open {url} in a browser instead)")
        return
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"(stream not opening: {url} — check WiFi / try browser)")
        return
    print(f"(video: {url} — press q in the window to close)")
    while not stop.is_set():
        ok, img = cap.read()
        if not ok:
            time.sleep(0.5)
            cap.release()
            cap = cv2.VideoCapture(url)
            continue
        cv2.imshow("robot-cam", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="robot-cam.local")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--send", default=None,
                    help="send one command, print replies for 2 s, exit")
    args = ap.parse_args()

    ws_url = f"ws://{args.host}:81/"
    stream_url = f"http://{args.host}/stream"
    stop = threading.Event()

    try:
        ws = websocket.create_connection(ws_url, timeout=5)
    except Exception as e:
        print(f"cannot reach {ws_url}: {e}", file=sys.stderr)
        return 1
    ws.settimeout(0.5)
    print(f"connected: {ws_url}")

    def reader() -> None:
        while not stop.is_set():
            try:
                msg = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except Exception:
                if not stop.is_set():
                    print("(command channel closed)")
                return
            if msg:
                print(msg if isinstance(msg, str) else msg.decode(errors="replace"))

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    if args.send is not None:
        ws.send(frame(args.send))
        time.sleep(2.0)
        stop.set()
        ws.close()
        return 0

    vt = None
    if not args.no_video:
        vt = threading.Thread(target=video_loop, args=(stream_url, stop),
                              daemon=True)
        vt.start()

    print("type commands ('h' help, 'x' E-STOP, Ctrl-C quits):")
    try:
        for line in sys.stdin:
            cmd = line.strip()
            if cmd:
                ws.send(frame(cmd))
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot safe if we were the active controller
        try:
            ws.send(frame("x"))
        except Exception:
            pass
        stop.set()
        time.sleep(0.3)
        ws.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
