import numpy as np
import time
import json
import redis
import math
import argparse
import os
import xml.etree.ElementTree as ET
from enum import Enum, auto
from dataclasses import dataclass

DEG_TO_RAD = math.pi / 180.0
OPENSAI_ROBOT_FILES_DIR = "/home/src1/OpenSai/config_folder/robot_files"

class State(Enum):
    RESETTING_JOINTS = auto()
    IDLE = auto()
    JOLT_UP = auto()
    JOLT_DOWN = auto()


real_robot_name = "Titania"
simulation_robot_name = "Rizon4r"

real_robot_config_file = "single_rizon_real.xml"
simulation_config_file = "single_rizon_vis.xml"
parser = argparse.ArgumentParser()
parser.add_argument("--real", action="store_true", help="Run against the real robot instead of simulation.")
args = parser.parse_args()

ENV = "real" if args.real else "simulation"
if ENV == "real":
    robot_name = "Titania"
    config_file_for_this_example = real_robot_config_file
    robot_model_file = os.path.join(OPENSAI_ROBOT_FILES_DIR, "Rizon4R.urdf")
else:
    robot_name = "Rizon4r"
    config_file_for_this_example = simulation_config_file
    robot_model_file = os.path.join(OPENSAI_ROBOT_FILES_DIR, "Rizon4r_with_kendama_parallel.urdf")

@dataclass
class RedisKeys:
    cartesian_task_goal_position: str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::goal_position"
    cartesian_task_goal_orientation: str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::goal_orientation"
    cartesian_task_current_position: str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::current_position"
    cartesian_task_current_orientation: str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::current_orientation"
    joint_task_goal_position: str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::goal_position"
    joint_task_goal_velocity: str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::goal_velocity"
    joint_task_goal_acceleration: str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::goal_acceleration"
    joint_task_current_position: str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::current_position"
    joint_sensor_position: str = f"opensai::sensors::{robot_name}::joint_positions"
    ball_pose: str = "opensai::sensors::KendamaBall::object_pose"
    ball_velocity: str = "opensai::sensors::KendamaBall::object_velocity"
    active_controller: str = f"opensai::controllers::{robot_name}::active_controller_name"
    config_file_name: str = "::sai-interfaces-webui::config_file_name"

redis_keys = RedisKeys()

joint_controller = "joint_controller"
cartesian_controller = "cartesian_controller"

# Joint position with end-effector pointing downward
# Joint 2 and 4 at -90 degrees to point end-effector straight down
LOWERED_START_JOINT_POS_PERP_Y = np.array([
    0.0,
    0.0,
    0.0,
    1.57079632679,
    0.0,
    0.0,
    0.0,
])

LOWERED_START_JOINT_POS_PARALLEL_Y = np.array([
    0.0,
    0.7853981633974483,
    0.0,
    -1.57079632679,
    0.0,
    0.7853981633974483,
    1.57079632679,
])

default_joint_pos = LOWERED_START_JOINT_POS_PARALLEL_Y

# Motion + timing parameters
joint_arrival_threshold = 0.10    # rad max per-joint error before leaving reset
reset_timeout = 35.0              # seconds; real reset aborts on timeout
reset_ramp_duration = 20.0        # seconds; avoid a large joint-space step on hardware
idle_hold_duration = 1.0
jolt_height = 0.01         # meters cup rises during JOLT_UP
jolt_up_duration = 0.60     # seconds spent commanding the upward target
landing_dip = 0.005          # meters cup dips below rest to damp landing
jolt_down_duration = 0.60   # seconds to hold the dip
settle_duration = 1.0       # seconds to blend back to rest height
auto_repeat = False         # set True to loop continuously

# Pose bookkeeping
rest_cup_pos = None
rest_cup_ori = None
reset_start_joint_pos = None
flange_fk_chain = None

# State tracking
state = State.RESETTING_JOINTS
state_entry_time = 0.0

cycle_requested = True  # run one jolt sequence after reset by default

# redis client
redis_client = redis.Redis()

# check that the config file is correct
config_file_name = redis_client.get(redis_keys.config_file_name).decode("utf-8")
if config_file_name != config_file_for_this_example:
    print("This example is meant to be used with the config file: ", config_file_for_this_example)
    print("But instead you have: ", config_file_name)
    exit(0)

if default_joint_pos is None:
    default_joint_pos = LOWERED_START_JOINT_POS.copy()
    print("Default joint pose not set; using lowered start pose.")
else:
    print("Using lowered start joint pose:", default_joint_pos)


def set_cartesian_goal(position, orientation):
    redis_client.set(
        redis_keys.cartesian_task_goal_position,
        json.dumps(np.asarray(position).tolist()),
    )
    redis_client.set(
        redis_keys.cartesian_task_goal_orientation,
        json.dumps(np.asarray(orientation).tolist()),
    )


def set_joint_goal(position):
    redis_client.set(redis_keys.joint_task_goal_position, json.dumps(position.tolist()))
    redis_client.set(
        redis_keys.joint_task_goal_velocity,
        json.dumps(np.zeros_like(position).tolist()),
    )
    redis_client.set(
        redis_keys.joint_task_goal_acceleration,
        json.dumps(np.zeros_like(position).tolist()),
    )


def rotation_from_rpy(rpy):
    roll, pitch, yaw = rpy

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rot_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])
    rot_y = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])
    rot_z = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])

    return rot_z @ rot_y @ rot_x


def rotation_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0.0:
        raise RuntimeError("URDF joint axis has zero length")

    x, y, z = axis / axis_norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c

    return np.array([
        [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
    ])


def homogeneous_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def parse_float_triplet(value, default):
    if value is None:
        return np.array(default, dtype=float)
    return np.array([float(item) for item in value.split()], dtype=float)


def load_urdf_chain_to_tip(urdf_path, tip_link):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints_by_child = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]

        origin = joint.find("origin")
        xyz = parse_float_triplet(origin.attrib.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
        rpy = parse_float_triplet(origin.attrib.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])

        axis = joint.find("axis")
        axis_xyz = parse_float_triplet(axis.attrib.get("xyz") if axis is not None else None, [0.0, 0.0, 1.0])

        joints_by_child[child] = {
            "name": joint.attrib["name"],
            "type": joint.attrib["type"],
            "parent": parent,
            "child": child,
            "origin_xyz": xyz,
            "origin_rpy": rpy,
            "axis": axis_xyz,
        }

    chain = []
    current_link = tip_link
    while current_link in joints_by_child:
        joint = joints_by_child[current_link]
        chain.append(joint)
        current_link = joint["parent"]

    if not chain:
        raise RuntimeError(f"Could not find kinematic chain to link {tip_link} in {urdf_path}")

    chain.reverse()
    actuated_joints = [joint for joint in chain if joint["type"] in ("revolute", "continuous", "prismatic")]
    if len(actuated_joints) != 7:
        raise RuntimeError(
            f"Expected 7 actuated joints in chain to {tip_link}, found {len(actuated_joints)}"
        )

    return chain


def compute_flange_pose_from_joints(joint_positions):
    global flange_fk_chain

    if flange_fk_chain is None:
        flange_fk_chain = load_urdf_chain_to_tip(robot_model_file, "flange")

    joint_positions = np.asarray(joint_positions, dtype=float)
    if joint_positions.shape[0] != 7:
        raise RuntimeError(f"Expected 7 joint positions for FK, got {joint_positions.shape[0]}")

    transform = np.eye(4)
    joint_index = 0
    for joint in flange_fk_chain:
        origin_rotation = rotation_from_rpy(joint["origin_rpy"])
        transform = transform @ homogeneous_transform(origin_rotation, joint["origin_xyz"])

        joint_type = joint["type"]
        if joint_type in ("revolute", "continuous"):
            transform = transform @ homogeneous_transform(
                rotation_from_axis_angle(joint["axis"], joint_positions[joint_index]),
                [0.0, 0.0, 0.0],
            )
            joint_index += 1
        elif joint_type == "prismatic":
            transform = transform @ homogeneous_transform(
                np.eye(3),
                joint["axis"] * joint_positions[joint_index],
            )
            joint_index += 1

    return transform[:3, 3].copy(), transform[:3, :3].copy()


def set_active_controller(controller_name):
    while redis_client.get(redis_keys.active_controller).decode("utf-8") != controller_name:
        redis_client.set(redis_keys.active_controller, controller_name)
        time.sleep(0.001)


def get_json_array(key):
    value = redis_client.get(key)
    if value is None:
        raise RuntimeError(f"Missing Redis key: {key}")
    try:
        return np.array(json.loads(value))
    except (TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Could not parse Redis key {key}: {value!r}") from exc


def get_current_joint_position():
    if ENV == "real":
        return get_json_array(redis_keys.joint_sensor_position)
    return get_json_array(redis_keys.joint_task_current_position)


def interpolate_joint_reset_goal():
    if reset_start_joint_pos is None:
        return default_joint_pos

    alpha = min(1.0, loop_time / reset_ramp_duration)
    alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha)
    return reset_start_joint_pos + alpha_smooth * (default_joint_pos - reset_start_joint_pos)


def enter_cartesian_hold_from_current_pose():
    active = redis_client.get(redis_keys.active_controller)
    if active is None:
        raise RuntimeError(f"Missing Redis key: {redis_keys.active_controller}")

    active = active.decode("utf-8")
    if active != joint_controller:
        raise RuntimeError(
            f"Expected {joint_controller} before Cartesian transition, got {active}"
        )

    current_joint_position = get_current_joint_position()
    current_pos, current_ori = compute_flange_pose_from_joints(current_joint_position)

    set_cartesian_goal(current_pos, current_ori)
    time.sleep(0.1)

    set_active_controller(cartesian_controller)
    set_cartesian_goal(current_pos, current_ori)

    return current_pos, current_ori


def seed_cartesian_hold_if_active():
    active = redis_client.get(redis_keys.active_controller)
    if active is None:
        raise RuntimeError(f"Missing Redis key: {redis_keys.active_controller}")

    active = active.decode("utf-8")
    if active != cartesian_controller:
        return

    current_pos = get_json_array(redis_keys.cartesian_task_current_position)
    current_ori = get_json_array(redis_keys.cartesian_task_current_orientation)
    set_cartesian_goal(current_pos, current_ori)


# loop at 200 Hz
loop_time = 0.0
dt = 0.005

time.sleep(0.01)
init_time = time.perf_counter_ns() * 1e-9

print("=" * 60)
print("KENDAMA JOLT CONTROLLER")
print("=" * 60)

# Seed the joint task at the measured pose before activating joint control.
seed_cartesian_hold_if_active()
reset_start_joint_pos = get_current_joint_position()
set_joint_goal(reset_start_joint_pos)
set_active_controller(joint_controller)

print("Resetting to default joint position...")
reset_start_time = loop_time
last_reset_print_time = -1.0


try:
    while True:
        loop_time += dt
        time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))

        if state == State.RESETTING_JOINTS:
            reset_goal = interpolate_joint_reset_goal()
            set_joint_goal(reset_goal)

            current_joint_position = get_current_joint_position()

            joint_errors = np.abs(default_joint_pos - current_joint_position)
            joint_error = np.max(joint_errors)

            if loop_time - last_reset_print_time > 0.5:
                print(f"Reset max joint error: {joint_error:.4f}")
                last_reset_print_time = loop_time

            reset_timed_out = (loop_time - reset_start_time) > reset_timeout

            if joint_error < joint_arrival_threshold or reset_timed_out:
                if reset_timed_out and ENV == "real":
                    raise RuntimeError(
                        f"Reset timed out on real robot with max_joint_error={joint_error:.4f}; "
                        "staying out of Cartesian mode."
                    )
                if reset_timed_out:
                    print(f"Reset timeout reached with max_joint_error={joint_error:.4f}; continuing.")
                else:
                    print("Default joint position reached. Capturing cup pose and going idle.")
                rest_cup_pos, rest_cup_ori = enter_cartesian_hold_from_current_pose()
                print(f"kendama_big_cup rest position: {rest_cup_pos.tolist()}")
                print(f"kendama_big_cup rest orientation:\n{rest_cup_ori}")
                state = State.IDLE
                state_entry_time = loop_time

        elif state == State.IDLE:
            if rest_cup_pos is None:
                continue
            set_cartesian_goal(rest_cup_pos, rest_cup_ori)
            if cycle_requested and (loop_time - state_entry_time) > idle_hold_duration:
                cycle_requested = auto_repeat  # prevent auto-repeat unless requested
                state = State.JOLT_UP
                state_entry_time = loop_time
                print("Starting JOLT_UP.")

        elif state == State.JOLT_UP:
            # Smooth vertical ramp upward instead of an abrupt step.
            if rest_cup_pos is None:
                continue
            elapsed = loop_time - state_entry_time
            alpha = min(1.0, elapsed / jolt_up_duration)

            # Smoothstep easing: zero slope at start and end.
            alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha)

            target_z = rest_cup_pos[2] + alpha_smooth * jolt_height
            up_goal = np.array([rest_cup_pos[0], rest_cup_pos[1], target_z])
            set_cartesian_goal(up_goal, rest_cup_ori)

            if elapsed > jolt_up_duration:
                state = State.JOLT_DOWN
                state_entry_time = loop_time
                print("Switching to JOLT_DOWN for landing dampening.")

        elif state == State.JOLT_DOWN:
            # Stay on the same XY column; only move along world Z.
            if rest_cup_pos is None:
                continue
            elapsed = loop_time - state_entry_time
            dip_z = rest_cup_pos[2] - landing_dip
            if elapsed < jolt_down_duration:
                target_z = dip_z
            elif elapsed < jolt_down_duration + settle_duration:
                alpha = (elapsed - jolt_down_duration) / settle_duration
                target_z = dip_z * (1 - alpha) + rest_cup_pos[2] * alpha
            else:
                print("Landing dampened. Returning to IDLE.")
                state = State.IDLE
                state_entry_time = loop_time
                continue
            target = np.array([rest_cup_pos[0], rest_cup_pos[1], target_z])
            set_cartesian_goal(target, rest_cup_ori)

except KeyboardInterrupt:
    print("\nKeyboard interrupt - stopping controller")
    pass
except Exception as e:
    print(f"\nError occurred: {e}")
    import traceback
    traceback.print_exc()
    pass
