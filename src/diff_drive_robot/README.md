# SmartBOT — LLM-Powered Autonomous Exploration Bot

---

## Problem Statement

Autonomous mobile robots are built in layers. Before a robot can be trusted to operate on its own, it needs to **move safely**, **explore the unknown**, and **understand what a person actually wants** — without anyone touching a coordinate or a frame name.

This project builds that robot one layer at a time: drive it by hand, teach it to navigate on its own, let it explore a space it's never seen, and finally hand control over to a language model so a plain-text command is enough to send it anywhere on the map.

---

## The Story: From Joystick to Plain Text

A robot is dropped into a maze it has never seen.

At first, it can't do anything by itself — you have to drive it, manually, with the keyboard, just to prove it can move at all.

Then you teach it to drive itself: given a goal point, it should reach it without you touching a key, swerving around whatever gets in its way.

Once it can do that reliably, you let it go further — you stop giving it goals entirely, and it has to decide for itself where the unexplored parts of the map are, drive there, and keep going until there's nothing left to discover.

Finally, the robot stops needing coordinates altogether. You type *"go to room_a"* — and somewhere between your sentence and the robot's wheels, a language model reads your words, looks up where "room_a" actually is, and sends the robot there.

That is the arc of this project: **teleoperation → navigation → exploration → LLM control.**

---

## Objective

Build a complete autonomous **ROS 2 Jazzy** pipeline on top of **Nav2**, **SLAM Toolbox**, and **Gazebo Harmonic**, capable of:

- Driving the robot manually over a ROS 2 topic
- Navigating to a goal point while avoiding obstacles, with no Nav2 required
- Exploring an unknown map on its own using frontier-based exploration
- Accepting plain-English commands and turning them into navigation goals via a local LLM

**Overall pipeline:**

```
Teleoperate → Navigate → Explore → Map → Type a command → LLM parses it → Robot goes there
```

---

## System Overview

The project consists of four major components, each building on the one before it.

### 1. Teleoperation
A minimal keyboard-to-`Twist` node. Proves the robot can move and that your sim, bridge, and topics are wired correctly before anything autonomous is attempted.

### 2. Custom Navigation
A standalone obstacle-avoidance navigator that does **not** use Nav2. It reads `/scan` and `/odom` directly and drives the robot to a goal point using a finite-state machine: seek the goal, detect an obstacle, find a clear heading, move past it, realign, repeat.

### 3. Frontier Exploration
Once SLAM Toolbox is mapping the environment, the robot has to decide *where* to go next without being told. The frontier explorer scans the live occupancy grid for the boundary between known free space and unknown space, picks the nearest unvisited frontier, and sends it to Nav2 as a goal — repeating until no frontiers remain.

### 4. LLM Navigation
A node that takes plain-English text (typed or published to a topic), sends it to a local Ollama model with a small instruction prompt, parses the model's JSON response, resolves it against a list of named locations, and sends the result to Nav2 as a `NavigateToPose` goal.

---

## Download ROS package

To download the starter package, clone the following git repo with the `starter-branch` into your colcon workspace:

```bash
git clone -b starter-branch https://github.com/RRoy4/LLM-Controlled-Bot.git
```

---

## What You Need To Implement

There are **5 files with TODOs** to complete. Each one already exists in the repo you've cloned, inside `scripts/`. 

| # | File | What you implement |
|---|---|---|
| 1 | `scripts/keyboard_teleop.py` | Keypress → `Twist` |
| 2 | `scripts/navigation.py` | Obstacle detection, clear-direction search, navigation FSM |
| 3 | `scripts/frontier_explorer.py` | Frontier detection, frontier selection |
| 4 | `scripts/llm_nav.py` | System prompt, Ollama API call, JSON extraction, goal resolution, goal dispatch, result handling |
| 5 | `scripts/obstacle_tracker.py` | Closing-ray detection, point clustering |

Three other files in `scripts/` are **complete, working reference implementations** you don't need to modify, but are worth reading once you've finished the corresponding TODO file above — they solve a related problem a different way, and comparing your approach to theirs is a good way to sanity-check your own design:

- **`pid_controller.py`** — solves the same goal-seeking problem as `navigation.py`, but with two independent PID loops (heading + distance) instead of a state machine. Worth running side-by-side with your finished `navigation.py` to compare how an FSM and a PID controller each handle the same goal.
- **`path_planning.py`** — a complete A* implementation (`ImprovedAStar`), demonstrating grid-based path planning with 8-direction movement and an obstacle safety margin. Note it currently plans against a small hardcoded obstacle list and a fixed start/goal, not the live map — useful as an A* reference, not a drop-in replacement for `frontier_explorer.py`.
- **`waypoint_nav.py`** — a working waypoint follower built on Nav2's `FollowWaypoints` action, with support for named locations (resolved from `config/locations.yaml`), raw coordinates, and patrol/loop mode. Useful to look at once you've populated `locations.yaml` for the LLM navigation step — it reads the same file.

### 1. Teleoperation — `scripts/keyboard_teleop.py`

**TODO 1 — Keypress to Twist**

Read keypresses from the terminal and convert them into linear/angular velocity commands published on `/cmd_vel`. Standard WASD (or arrow-key) mapping is expected, with a clean stop on key release and on exit.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze
ros2 run diff_drive_robot keyboard_teleop.py
```
Drive in all directions and confirm the robot stops cleanly the moment you release a key. You can also run `ros2 topic echo /cmd_vel` in a separate terminal to confirm the published values match what you'd expect for each key.

---

### 2. Custom Navigation — `scripts/navigation.py`

**TODO 1 — Front Obstacle Detection**

Using the live `/scan` data, compute the minimum distance to anything directly in front of the robot within a configurable angle.

**TODO 2 — Clear Direction Search**

When an obstacle is detected, scan a range of headings around the robot and find the direction with the most clearance that still moves the robot roughly toward the goal.

**TODO 3 — Navigation State Machine**

Implement the four-state FSM (`GOAL_SEEK → FIND_CLEAR → MOVE_CLEAR → REALIGN`) that ties detection and direction-finding together into continuous goal-reaching behavior, publishing `Twist` commands every cycle.

> Once this is working, take a look at `scripts/pid_controller.py` — a complete reference implementation that solves the same goal-seeking problem using two independent PID loops (heading and distance) instead of an FSM. Comparing the two is a good way to see how control-strategy choice affects behavior.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=obstacles
ros2 run diff_drive_robot navigation.py --ros-args -p goal_x:=3.0 -p goal_y:=2.0
```
Confirm the robot drives toward the goal, steers around any obstacle in its path without colliding, and comes to a stop once it reaches the goal. Try a few different `goal_x`/`goal_y` values and a couple of obstacle layouts before moving on.

---

### 3. Frontier Exploration — `scripts/frontier_explorer.py`

**TODO 1 — Frontier Detection**

Given the live `/map` occupancy grid, find all **frontier cells** — free cells directly adjacent to unknown cells — and cluster them into connected regions large enough to be worth visiting.

**TODO 2 — Frontier Selection**

From the detected frontier clusters, pick the nearest one the robot hasn't already visited, using the robot's live pose from TF, and send it to Nav2 as a `NavigateToPose` goal. Repeat until no valid frontiers remain.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true
```
Open RViz and watch the occupancy grid — the robot should start moving toward unexplored regions roughly 12 seconds after launch, with no goals given by you. Confirm it keeps picking new frontiers as old ones get explored, and that it stops cleanly once the whole map is filled in (rather than looping or getting stuck on a tiny leftover frontier).

---

### 4. Obstacle Tracker — `scripts/obstacle_tracker.py`

A moving-obstacle detector that compares consecutive laser scans to find objects approaching the robot, clusters the detected points in the map frame, and publishes them as RViz markers.

**TODO 1 — Closing-ray detection**

On each incoming `/scan`, compare it against a scan from `self._lookback` frames ago. For each ray, compute the closing speed (`(r_prev - r_now) / dt`). If it exceeds `self._min_speed`, the ray endpoint (in robot frame) is an approaching point. Collect all such points, transform them into the map frame using TF, then pass the result to `_cluster()`.

**TODO 2 — Point clustering**

Given a flat list of `(x, y)` map-frame points, group them using single-linkage clustering: any two points within `self._cluster_r` metres of each other belong to the same cluster. Return a list of dicts, one per cluster, each with the cluster centroid `x`, `y`, and `count` of member points.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=obstacles
ros2 run diff_drive_robot obstacle_tracker.py
```
In a separate terminal, echo the state topic and confirm clusters appear when something moves in front of the robot:
```bash
ros2 topic echo /obstacle_tracker/state
```
Open RViz and add a `MarkerArray` display on `/obstacle_tracker/markers` — you should see red spheres appear at the location of any moving object and disappear once it stops.

---

### 5. LLM Navigation — `scripts/llm_nav.py`

**TODO 1 — System prompt**

Write the `_SYSTEM` string that instructs the LLM to return only a JSON object. It must handle four cases — named location, raw coordinates, stop, and anything else — and include format placeholders `{locations}` and `{command}` which get filled in at runtime. Include at least two examples in the prompt so the model has concrete patterns to follow. The quality of this prompt directly determines whether the model returns parseable JSON or garbage — experiment with it.

**TODO 2 — Ollama API call**

Implement `call_ollama()`. Send a POST request to `{base_url}/api/generate` with the model name, prompt, `stream: false`, and `format: "json"` in the JSON body. Parse the response and return the model's text from `response["response"]`. Use only stdlib (`urllib.request`, `json`) — no third-party HTTP libraries.

**TODO 3 — JSON extraction**

Implement `_extract_json()`. The LLM will often wrap its JSON in prose or markdown even when told not to. Use a regex to find the first `{...}` block in the raw output string, then parse and return it. Return `None` if nothing valid is found.

**TODO 4 — Goal resolution**

Given the parsed JSON dict (one of `stop`, `unknown`, `go` with a location name, or `go` with raw coordinates), resolve it into an `(x, y, yaw_deg)` tuple. Look up named locations in `self._locations`, cancel goals on stop, log warnings on unknown actions, and return `None` for anything malformed.

**TODO 5 — Goal dispatch**

Implement `_send_goal()` and `_send_goal_thread()`. Dispatch the goal in a background daemon thread so the ROS executor never blocks. In the thread, wait for the Nav2 action server (up to 60s), build a `PoseStamped` using `_yaw_to_quat()`, store `self._goal_xy` and `self._nav_start_time`, and send the goal with `_goal_accepted_cb` and `_feedback_cb` registered.

**TODO 6 — Result handling**

Implement `_result_cb()`. Check the goal status, log success or failure, compute the Euclidean distance between `self._goal_xy` and `self._current_pose` and log it as accuracy, include elapsed time and recovery count in the log. Always clear `self._busy` at the end.

> `config/locations.yaml` ships with the key structure in place but no coordinates filled in. You'll populate it yourself once you've explored and mapped the world — this is intentional.

> ⚠️ **Keep commands simple.** This setup runs a small local model (`tinyllama`) through Ollama, not a hosted LLM — it has limited capacity for parsing complex or multi-part instructions. Stick to simple, single-intent commands like `go to room_a` or `go to 2.5 1.0`. Long, compound, or oddly phrased commands ("first go to room_a, then circle back to the kitchen if it's clear") are likely to cause the model to time out or return something `_resolve_goal()` can't parse.

**✅ How to check:**
```bash
# Start nav stack on the maze world — map auto-loads as map_maze.yaml if you saved it under that name
ros2 launch diff_drive_robot robot.launch.py world_name:=map

# Separate terminal — start the LLM navigator
ros2 run diff_drive_robot llm_nav.py
```
Try a named location (`go to room_a`), raw coordinates (`go to 2.5 1.0`), and something invalid (a typo'd location name, or gibberish text) — confirm the valid commands send the robot to the right place and the invalid one is rejected cleanly instead of crashing the node or sending a bogus goal.

> `robot.launch.py` takes a full path via `world:=`, not the short `world_name:=` shorthand used in the exploration step — but it auto-discovers your saved map (`map_<world_name>.yaml`) as long as you used `map_saver_cli -f .../map_maze` like in the exploration check above, so you don't need to pass `map:=` by hand.

---

## Running the Project

> **Before opening any terminal, build the workspace:**
> ```bash
> cd ~/rosnav
> colcon build
> ```
>
> **For every new terminal, source the workspace:**
> ```bash
> source ~/rosnav/install/setup.bash
> ```

---

### Prerequisites

```bash
# ROS 2 Jazzy (Ubuntu 24.04)
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard
```

**Ollama (for LLM navigation):**
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama serve &
ollama pull tinyllama
```

> Run order and exact commands for each stage are in the **✅ How to check** block right after that stage's TODOs above. One thing not covered there: once exploration finishes, save the map before moving on to LLM navigation:
> ```bash
> ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_maze
> ```

---

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs along with comments describing what you are doing
- Working ROS 2 package, building cleanly with `colcon build`
- All provided launch files functioning unmodified

### 2. Demonstration Video
Show, in order:
- Keyboard teleoperation
- Custom navigation reaching a goal and avoiding an obstacle
- Frontier exploration completing a full map
- Obstacle tracker detecting a moving object (red spheres visible in RViz)
- LLM navigation responding correctly to at least three different plain-English commands

### 3. Report
Your report should demonstrate a deep understanding of the algorithms implemented and the architectural decisions made. You are also encouraged to attach relevant images, flowcharts, and RViz screenshots to visually support your explanations. Break your report down into the following sections, explaining not just *what* code you wrote, but *how* it works and *why* it behaves the way it does:

*   **Teleoperation & Base Control:**
    *   Briefly explain your approach to mapping keystrokes to `Twist` messages. 
    *   Discuss the "clean stop" requirement: why is it critical for robotic safety, and what exactly happens in the simulation if velocity commands are left hanging upon key release?

*   **Custom Navigation (FSM Analysis):**
    *   Detail your Finite State Machine design. What exact mathematical or sensor conditions trigger the transitions between `GOAL_SEEK`, `FIND_CLEAR`, `MOVE_CLEAR`, and `REALIGN`?
    *   **Parameter Tuning:** What happens to the robot's behavior if you increase or decrease the front obstacle detection angle? How does altering the "clearance" threshold change the robot's pathing around objects?
    *   Compare your FSM implementation to the provided `pid_controller.py` reference. In your observation, what are the pros and cons of using an FSM versus independent PID loops for obstacle avoidance?

*   **Frontier Exploration & Mapping:**
    *   Walk through your algorithm for finding and grouping frontier cells from a 1D occupancy grid array.
    *   **Parameter Tuning:** What happens if the minimum frontier cluster size is set too small? What if it is set too large?
    *   Explain your selection logic. You were asked to pick the *nearest* frontier—what would visually change in the exploration pattern if the robot always picked the *largest* frontier instead?

*   **Dynamic Obstacle Tracking:**
    *   Explain the logic and math behind your closing-ray detection. 
    *   **Parameter Tuning:** Discuss the impact of tweaking `_lookback` frames and `_min_speed`. What happens to your tracker if `_lookback` is set to 1 frame versus 20 frames? What are the tradeoffs?
    *   Describe your single-linkage clustering implementation. How does changing the distance threshold (`_cluster_r`) affect the number of RViz markers generated for a single moving object?

*   **LLM Integration & Prompt Engineering:**
    *   Include your final `_SYSTEM` prompt in the report. 
    *   Document your prompt engineering process. What early prompt versions failed? Why did the model output unparseable text, and how did your final prompt structure fix it?
    *   Explain your regex approach for JSON extraction. 
    *   Discuss edge-case handling: How does your goal resolution logic gracefully handle hallucinations (e.g., the LLM making up a room name that isn't in `locations.yaml`)?


---

## Final Message

Congratulations — if you've made it through all four stages, you've built a robot that can be driven by hand, navigate on its own, explore a space it's never seen, and respond to plain-text commands by reading a map it built itself. That's a complete autonomy stack, end to end, and it's no small thing to have working.

If you want to keep going, this project has plenty of room to grow:

- **Smarter LLM parsing** — handle multi-step commands ("go to room_a, then room_b"), relative instructions, or confirmation prompts before executing a goal.
- **Better exploration** — add frontier-size weighting, information-gain scoring, or a smarter revisit policy so the robot explores more efficiently.
- **Tighter control** — swap the FSM in `navigation.py` for a smoother controller, or tune the PID gains in `pid_controller.py` for faster, less jerky goal-seeking.
- **New environments** — try `warehouse`, `house`, or `corridor` instead of `maze`, and see what breaks.
- **Voice input** — feed `llm_nav.py` from a speech-to-text pipeline instead of typed text, so commands can be spoken instead of typed.
- **Multiple robots** — extend the stack to coordinate more than one robot exploring or navigating the same map.

Pick whichever direction is most interesting to you and keep building.
