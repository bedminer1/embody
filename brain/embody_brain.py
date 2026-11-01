#!/usr/bin/env python3
"""Claude plans, the desktop app drives: type a task, the rover does it.

Claude gets coarse tools (drive 3 m, turn, go to a remembered place, look)
and is called once per step of the task, not once per few centimetres. Each
tool call runs a closed-loop skill in the desktop app (`--json` mode) on the
RVR's own odometry and returns only when the rover has arrived, failed or
been stopped.

    cd desktop && cargo build --release --locked
    python3 brain/embody_brain.py --direct <esp-ip>           # ESP32 agent
    python3 brain/embody_brain.py --target pi5                # Pi over SSH
    python3 brain/embody_brain.py --direct 127.0.0.1:8765     # tools/sim_rover.py

At the prompt type a task, or /stop, /reset (pose and home here), /status,
/quit. Ctrl-C while the rover is moving stops it.
"""
import argparse
import base64
import itertools
import json
import queue
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import anthropic

ROOT = Path(__file__).resolve().parent.parent
MODEL = "claude-opus-5-5"

SYSTEM = """You control a Sphero RVR ground rover indoors. You plan; the rover's \
own controller executes. Each movement tool runs to completion (arrival, \
failure or user stop) before it returns, so give whole moves: one drive of \
3 m, not many short ones.

Coordinates come from the rover's wheel and IMU odometry, in metres: +y is \
the direction the rover faced at the last pose reset ("home"), +x is to the \
right of that, yaw is in degrees, counter-clockwise (left) positive, 0 = \
facing +y. Odometry drifts a few percent of the distance travelled and more \
after many turns, so remembered places get less precise over a long session.

The rover has no obstacle sensing. A move fails with "no progress" when it \
is blocked or slipping; assume an obstacle, back off and go around. If a \
camera is available, use look before driving into space you have not seen, \
and after a failure. Without a camera, move cautiously and ask the user \
when unsure. Keep speeds at or below 0.3 m/s unless told otherwise.

Reply briefly. When the task is done or impossible, say what happened and \
where the rover is."""


def tool(name, description, properties=None, required=()):
    return {
        "name": name,
        "description": description,
        "input_schema": {
            "type": "object",
            "properties": properties or {},
            "required": list(required),
            "additionalProperties": False,
        },
    }


SPEED = {"type": "number", "description": "m/s, 0.06 to 0.6; default 0.3"}
TOOLS = [
    tool("drive", "Drive straight along the current heading, holding the line. "
         "Negative distance reverses. Up to 10 m per call.",
         {"distance_m": {"type": "number"}, "speed_mps": SPEED}, ["distance_m"]),
    tool("turn", "Turn in place by a relative angle; positive turns left (CCW).",
         {"degrees": {"type": "number"}}, ["degrees"]),
    tool("turn_to", "Turn in place to face an absolute yaw in degrees.",
         {"yaw_deg": {"type": "number"}}, ["yaw_deg"]),
    tool("go_to", "Turn toward and drive to a point in the odometry frame, "
         "optionally ending at a yaw. Up to 20 m away.",
         {"x": {"type": "number"}, "y": {"type": "number"},
          "yaw_deg": {"type": "number"}, "speed_mps": SPEED}, ["x", "y"]),
    tool("go_to_place", "Drive to a remembered place and face its saved heading.",
         {"name": {"type": "string"}, "speed_mps": SPEED}, ["name"]),
    tool("remember_place", "Save the rover's current pose under a name.",
         {"name": {"type": "string"}}, ["name"]),
    tool("forget_place", "Delete a remembered place.",
         {"name": {"type": "string"}}, ["name"]),
    tool("list_places", "List remembered places and their poses."),
    tool("get_status", "Current pose, battery and motor link state."),
    tool("look", "Take a picture with the rover's forward camera."),
]


class RoverGone(RuntimeError):
    """The desktop app exited, so nothing can drive the rover."""


class Rover:
    """The desktop app in `--json` mode, as a child process."""

    def __init__(self, command):
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.events = queue.Queue()
        self.telemetry = None
        self.ids = itertools.count(1)
        threading.Thread(target=self._read, daemon=True).start()
        self.ready = self.wait(lambda event: event["type"] == "ready", 20)
        if self.ready is None:
            raise RoverGone("The desktop app did not start; see its error above")

    def _read(self):
        for line in self.process.stdout:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("type") == "telemetry":
                self.telemetry = event["data"]
            else:
                self.events.put(event)
        self.events.put({"type": "exit"})

    def send(self, message):
        message.setdefault("id", next(self.ids))
        self.process.stdin.write(json.dumps(message) + "\n")
        self.process.stdin.flush()
        return message["id"]

    def wait(self, match, timeout):
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            try:
                event = self.events.get(timeout=remaining)
            except queue.Empty:
                break
            if event["type"] == "exit":
                raise RoverGone("The desktop app exited")
            if match(event):
                return event
        return None

    def request(self, message, timeout=5):
        request_id = self.send(message)
        return self.wait(lambda event: event.get("id") == request_id, timeout)

    def run_skill(self, skill):
        """Start a skill and block until it ends. Ctrl-C stops the rover."""
        request_id = self.send(skill)
        try:
            # Skills time out on their own; this is only a backstop.
            event = self.wait(lambda e: e.get("id") == request_id and e.get("type") == "skill"
                              and e["state"] in ("done", "failed", "cancelled"), 180)
        except KeyboardInterrupt:
            self.send({"stop": True})
            raise
        if event is None:
            self.send({"stop": True})
            return "No result from the rover after 180 s; stopped it."
        pose = event.get("pose")
        where = f" Pose now x={pose['x']:.2f} y={pose['y']:.2f} yaw={pose['yaw']:.0f}." if pose else ""
        return f"{event['state']}: {event.get('message', '')}.{where}"

    def close(self):
        if self.process.poll() is None:
            try:
                self.send({"quit": True})
                self.process.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()


def execute(rover, name, args):
    """Run one tool; returns tool_result content (a string or blocks)."""
    speed = {"speed_mps": args["speed_mps"]} if "speed_mps" in args else {}
    if name == "drive":
        return rover.run_skill({"skill": "drive", "distance_m": args["distance_m"], **speed})
    if name == "turn":
        return rover.run_skill({"skill": "turn", "degrees": args["degrees"]})
    if name == "turn_to":
        return rover.run_skill({"skill": "turn_to", "yaw_deg": args["yaw_deg"]})
    if name == "go_to":
        target = {key: args[key] for key in ("x", "y", "yaw_deg") if key in args}
        return rover.run_skill({"skill": "go_to", **target, **speed})
    if name == "go_to_place":
        return rover.run_skill({"skill": "go_to", "place": args["name"], **speed})
    if name == "remember_place":
        return json.dumps(rover.request({"remember": args["name"]}))
    if name == "forget_place":
        return json.dumps(rover.request({"forget": args["name"]}))
    if name == "list_places":
        return json.dumps(rover.request({"places": True}))
    if name == "get_status":
        status = rover.request({"status": True}) or {}
        data = rover.telemetry or {}
        return json.dumps({"pose": status.get("pose"), "skill": status.get("skill"),
                           "battery": data.get("battery"),
                           "rvr_connected": data.get("rvr_connected"),
                           "error": data.get("error")})
    if name == "look":
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "frame.jpg"
            event = rover.request({"snapshot": str(path)})
            if not event or event["type"] != "snapshot":
                raise RuntimeError((event or {}).get("message", "No camera frame"))
            data = base64.standard_b64encode(path.read_bytes()).decode()
        return [{"type": "image",
                 "source": {"type": "base64", "media_type": "image/jpeg", "data": data}},
                {"type": "text", "text": "Forward camera, just now."}]
    raise ValueError(f"Unknown tool {name}")


class Brain:
    def __init__(self, rover, effort):
        self.client = anthropic.Anthropic()
        self.rover = rover
        self.effort = effort
        self.messages = []

    def say(self, text):
        """Add the user's words, then run the tool loop until Claude stops."""
        # A Ctrl-C can leave unsent tool results last; the new text joins them.
        if self.messages and self.messages[-1]["role"] == "user":
            self.messages[-1]["content"].append({"type": "text", "text": text})
        else:
            self.messages.append({"role": "user", "content": [{"type": "text", "text": text}]})
        while True:
            response = self.client.beta.messages.create(
                model=MODEL,
                max_tokens=16000,
                system=SYSTEM,
                tools=TOOLS,
                messages=self.messages,
                thinking={"type": "adaptive"},
                output_config={"effort": self.effort},
                betas=["server-side-fallback-2026-07-01"],
                fallbacks="default",
            )
            self.messages.append({"role": "assistant", "content": response.content})
            for block in response.content:
                if block.type == "text" and block.text.strip():
                    print(f"\nrover> {block.text.strip()}")
            if response.stop_reason == "refusal":
                print("\n(The request was declined.)")
                return
            if response.stop_reason != "tool_use":
                return
            calls = [block for block in response.content if block.type == "tool_use"]
            results = []
            self.messages.append({"role": "user", "content": results})
            for call in calls:
                print(f"  · {call.name} {json.dumps(call.input)}", flush=True)
                try:
                    content, is_error = execute(self.rover, call.name, call.input), False
                except KeyboardInterrupt:
                    content, is_error = "Stopped by the user; the rover is stationary.", True
                    self._interrupted(calls, results, call, content)
                    print("\n  stopped")
                    return
                except RoverGone:
                    raise
                except Exception as error:  # reported to Claude, which can adapt
                    content, is_error = f"Error: {error}", True
                if isinstance(content, str):
                    print(f"    {content}")
                results.append({"type": "tool_result", "tool_use_id": call.id,
                                "content": content, "is_error": is_error})

    @staticmethod
    def _interrupted(calls, results, current, message):
        """Every tool_use needs a result, including those never started."""
        for call in calls[calls.index(current):]:
            text = message if call is current else "Not run: the user stopped the task."
            results.append({"type": "tool_result", "tool_use_id": call.id,
                            "content": text, "is_error": True})


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    link = parser.add_mutually_exclusive_group()
    link.add_argument("--direct", help="rover agent HOST[:PORT] (ESP32 or simulator)")
    link.add_argument("--target", help="SSH host of the Pi agent (default pi5)")
    parser.add_argument("--places", default=str(ROOT / "places.json"),
                        help="file that keeps named places between runs")
    parser.add_argument("--binary", default=str(ROOT / "desktop/target/release/embody-desktop"))
    parser.add_argument("--effort", default="medium",
                        choices=["low", "medium", "high", "xhigh", "max"])
    args = parser.parse_args()
    if not Path(args.binary).exists():
        sys.exit(f"{args.binary} not found: run `cargo build --release --locked` in desktop/")
    command = [args.binary, "--json", "--places", args.places]
    command += ["--direct", args.direct] if args.direct else ["--target", args.target or "pi5"]

    rover = Rover(command)
    brain = Brain(rover, args.effort)
    camera = "camera available" if rover.ready.get("video_available") else "no camera"
    print(f"Connected ({camera}). Places: {', '.join(rover.ready['places']) or 'none'}.")
    print("Type a task, or /stop /reset /status /quit.")
    try:
        while True:
            try:
                text = input("\nyou> ").strip()
            except KeyboardInterrupt:
                rover.send({"stop": True})
                print()
                continue
            if not text:
                continue
            if text == "/quit":
                break
            if text == "/stop":
                rover.send({"stop": True})
            elif text == "/reset":
                rover.send({"reset_pose": True})
                print("Pose reset: this spot and heading are now home (0, 0, 0°).")
            elif text == "/status":
                print(execute(rover, "get_status", {}))
            else:
                try:
                    brain.say(text)
                except KeyboardInterrupt:
                    rover.send({"stop": True})
                    print("\n  cancelled")
                except anthropic.APIConnectionError as error:
                    print(f"Cannot reach the Claude API: {error}")
                except anthropic.RateLimitError:
                    print("Rate limited by the Claude API; try again shortly.")
                except anthropic.APIStatusError as error:
                    print(f"Claude API error {error.status_code}: {error.message}")
    except EOFError:
        pass
    except RoverGone as error:
        print(error)
    finally:
        rover.close()


if __name__ == "__main__":
    main()
