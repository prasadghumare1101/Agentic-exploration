#!/usr/bin/env python3
"""Give every SITL vehicle its own ROS namespace for the gazebo_ros sensor plugins.

WHY THIS EXISTS
---------------
iris.sdf.jinja wraps each gazebo_ros plugin in <namespace>{{ ns }}</namespace>, where `ns`
comes from a `vehicle_namespace` template variable. Two stock PX4 files have to cooperate to
supply it, and neither does out of the box:

    scripts/jinja_gen.py    must accept --vehicle_namespace and pass it to the template
    sitl_multiple_run.sh    must send --vehicle_namespace <model>_<N> per vehicle

Without both, `vehicle_namespace` is never defined, the template falls back to '/', and every
vehicle registers its plugins at the root. A 2+ vehicle launch then creates several ROS nodes
with the same name and gzserver dies during spawn:

    [ERROR] [gazebo_ros_node]: Found multiple nodes with same name: /slam_imu_plugin
    sitl_multiple_run.sh: line 153: Segmentation fault (core dumped) gzserver ... .world

To the operator that looks like a black Gazebo window plus a stack where nothing connects,
with no obvious link back to a naming collision.

This runs AFTER the firmware build on purpose: both targets are runtime scripts, so patching
them here leaves the expensive PX4 compile layer cached.

Every edit is idempotent and anchored to an exact upstream line. If an anchor is missing the
script exits non-zero and fails the image build, so a future PX4 bump can never quietly
produce an image whose swarm mode segfaults.
"""
import sys
from pathlib import Path

PX4 = Path(sys.argv[1] if len(sys.argv) > 1 else "/opt/PX4-Autopilot")
GZ = PX4 / "Tools" / "simulation" / "gazebo-classic"

# (file, marker that means "already patched", anchor line, text inserted before the anchor)
EDITS = [
    (
        GZ / "sitl_gazebo-classic" / "scripts" / "jinja_gen.py",
        "--vehicle_namespace",
        "    parser.add_argument('--cam_component_id'",
        "    parser.add_argument('--vehicle_namespace', default='',"
        " help=\"Per-vehicle ROS namespace for gazebo_ros sensor plugins"
        " (empty = global/root)\")\n",
    ),
    (
        GZ / "sitl_gazebo-classic" / "scripts" / "jinja_gen.py",
        "'vehicle_namespace':",
        "         'cam_component_id': args.cam_component_id, \\",
        "         'vehicle_namespace': args.vehicle_namespace, \\\n",
    ),
    (
        GZ / "sitl_multiple_run.sh",
        "--vehicle_namespace",
        "\tset -- ${@} --gst_udp_port $((5600+${N}))",
        "\tset -- ${@} --vehicle_namespace ${MODEL}_${N}\n",
    ),
]


def apply(path: Path, marker: str, anchor: str, insert: str) -> str:
    if not path.is_file():
        raise SystemExit(f"px4-namespace-patch: missing {path}")
    text = path.read_text()
    if marker in text:
        return f"already patched ({marker})"
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.startswith(anchor):
            lines.insert(i, insert)
            path.write_text("".join(lines))
            return f"inserted before line {i + 1}"
    raise SystemExit(
        f"px4-namespace-patch: anchor not found in {path}\n"
        f"  looking for a line starting: {anchor!r}\n"
        "  PX4 has changed upstream; update docker/px4_namespace_patch.py."
    )


def main() -> None:
    for path, marker, anchor, insert in EDITS:
        print(f"  {path.name:<22} {apply(path, marker, anchor, insert)}")

    # Prove the two halves actually line up, rather than trusting the edits landed.
    jinja = (GZ / "sitl_gazebo-classic" / "scripts" / "jinja_gen.py").read_text()
    runner = (GZ / "sitl_multiple_run.sh").read_text()
    problems = []
    if "--vehicle_namespace" not in jinja:
        problems.append("jinja_gen.py does not accept --vehicle_namespace")
    if "'vehicle_namespace': args.vehicle_namespace" not in jinja:
        problems.append("jinja_gen.py does not forward vehicle_namespace to the template")
    if "--vehicle_namespace ${MODEL}_${N}" not in runner:
        problems.append("sitl_multiple_run.sh does not pass a per-vehicle namespace")
    if problems:
        raise SystemExit("px4-namespace-patch FAILED:\n  - " + "\n  - ".join(problems))
    print("  verified: per-vehicle ROS namespaces are wired end to end")


if __name__ == "__main__":
    main()
