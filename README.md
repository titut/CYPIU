# CYPIU — Can You Pick It Up?

An LLM-driven pick-and-place system for a **myCobot 280** 6-DOF arm on a Raspberry Pi: you type *"pick up the apple"*, and the robot detects the object, estimates its pose, plans joint angles, and grabs it.

Built as a ROS 2 package — and the project that taught me exactly where ROS 2 earns its keep and where it gets in the way. That experience is what motivated its successor, [ZenNav](https://github.com/titut/CYR), a from-scratch autonomy stack on Zenoh.

**Pipeline:** natural language → LLM parses intent → YOLOv4 finds the object → AprilTags + FK give world pose → inverse kinematics → joint angles → claw servo.

```
"pick up the apple"
        │
        ▼
┌──────────────┐   GPT-4.1-nano parses
│   cmd_gui    │   action + target object
└──────┬───────┘
       │ calls
       ▼
┌──────────────┐   YOLOv4 (ONNX, 416×416, COCO)
│ obj_detection│   finds "apple" in the camera frame
└──────┬───────┘
       ▼
┌──────────────┐   AprilTag (36h11) on the workspace + FK
│apriltag_svc  │   gives the tag frame; computes object
│  + ik/fk     │   pose, then damped least-squares IK
└──────┬───────┘   with line search → joint angles
       │ /joint_angles
       ▼
┌──────────────┐   pymycobot serial → real motors
│   movearm    │   + /current_angles feedback
└──────┬───────┘
       ▼
┌──────────────┐   gpiozero servo on GPIO 18
│    claw      │   open / close
└──────────────┘
```

## Hardware

| Part | Role |
|---|---|
| Elephant Robotics **myCobot 280** (6-DOF) | the arm, driven over serial (`/dev/ttyAMA0`) |
| Raspberry Pi + camera | on-board compute and perception |
| GPIO servo | claw gripper |
| PS5 controller | manual teleop fallback |

## Motion pipeline

- **Kinematics from scratch** on top of the `modern_robotics` library: the arm is modeled with the product-of-exponentials formalism (home configuration `M`, screw axes `S_list`), with forward kinematics verified against the physical arm
- **Inverse kinematics**: damped least-squares with a **line search** on step size — each candidate step is validated by running FK and checking the pose error norm before accepting it, which keeps the solver stable near singularities
- AprilTag detections are fused with FK-derived arm poses to get the object in the arm's frame; angles are clamped to joint limits before hitting hardware

## Perception

- **YOLOv4** exported to ONNX, run through `onnxruntime` (no GPU needed) at 416×416 with COCO classes — so "pick up the banana" resolves against real detections
- **AprilTags** (36h11) via `tf2_ros` for accurate world-frame localization of targets; camera intrinsics live in `config/camera_info.yaml`, tag geometry in `config/tags.yaml`
- Calibration tooling in `examples/camera_calibration.py`

## Nodes

| Node | What it does |
|---|---|
| `cmd_gui` | Takes natural-language commands, calls the LLM, orchestrates the pipeline |
| `obj_detection` | Subscribes to camera images, runs YOLOv4, overlays detections |
| `apriltag_service` | AprilTag detections + TF → object pose in arm frame → IK |
| `movearm` | Serial bridge to the myCobot 280; executes joint angles |
| `claw` | Servo open/close service |
| `teleop` | PS5 controller manual control |

## Quick start

```bash
# ROS 2 (Humble or newer) + dependencies
pip install -r requirements.txt
colcon build
source install/setup.bash

# each in its own terminal (or use the launch files)
ros2 run cypiu move            # arm driver
ros2 run cypiu apriltag_service
ros2 run cypiu obj_detection   # camera + YOLOv4
ros2 run cypiu cmd_gui         # type commands here
```

Launch files are in `launch/` (`camera_launch.xml`, `move_launch.xml`, `teleop_launch.xml`).

> YOLOv4 weights: the loader expects `~/yolo_model/yolov4.onnx` + `coco.names`.

## What this project taught me

This was my first end-to-end robotics build — CAD-free calibration, kinematics that survive contact with real hardware, and the failure modes of gluing an LLM onto a physical system (ambiguous commands, objects the detector has never seen, IK targets the arm physically can't reach).

It also taught me what ROS 2 costs: launch ceremony, `.msg` codegen, and QoS semantics I didn't fully control. That irritation became [**ZenNav**](https://github.com/titut/CYR) — the same ideas (pub/sub autonomy stack, layered safety, replay tooling) rebuilt by hand on Zenoh.

## License

Apache-2.0
