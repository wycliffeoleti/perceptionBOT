#!/usr/bin/env python3
"""
Spawn colored geometric objects into Gazebo for PerceptionBOT testing.

Usage:
    python3 scripts/spawn_objects.py [--count N] [--seed S]
"""
import argparse
import random
import subprocess
import textwrap

OBJECT_TYPES = {
    "red_cube": {
        "sdf_template": """
        <sdf version="1.9">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia><ixx>1.67e-5</ixx><ixy>0</ixy><ixz>0</ixz>
                         <iyy>1.67e-5</iyy><iyz>0</iyz><izz>1.67e-5</izz></inertia>
              </inertial>
              <collision name="collision">
                <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
              </collision>
              <visual name="visual">
                <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
                <material>
                  <ambient>0.9 0.1 0.1 1</ambient>
                  <diffuse>0.9 0.1 0.1 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>""",
        "z": 0.835,  # table_height + half_cube = 0.81 + 0.025
    },
    "blue_cube": {
        "sdf_template": """
        <sdf version="1.9">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia><ixx>1.67e-5</ixx><ixy>0</ixy><ixz>0</ixz>
                         <iyy>1.67e-5</iyy><iyz>0</iyz><izz>1.67e-5</izz></inertia>
              </inertial>
              <collision name="collision">
                <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
              </collision>
              <visual name="visual">
                <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
                <material>
                  <ambient>0.1 0.1 0.9 1</ambient>
                  <diffuse>0.1 0.1 0.9 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>""",
        "z": 0.835,
    },
    "green_cylinder": {
        "sdf_template": """
        <sdf version="1.9">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <inertial>
                <mass>0.08</mass>
                <inertia><ixx>2.5e-5</ixx><ixy>0</ixy><ixz>0</ixz>
                         <iyy>2.5e-5</iyy><iyz>0</iyz><izz>1.25e-5</izz></inertia>
              </inertial>
              <collision name="collision">
                <geometry><cylinder><radius>0.025</radius><length>0.07</length></cylinder></geometry>
              </collision>
              <visual name="visual">
                <geometry><cylinder><radius>0.025</radius><length>0.07</length></cylinder></geometry>
                <material>
                  <ambient>0.1 0.7 0.1 1</ambient>
                  <diffuse>0.1 0.7 0.1 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>""",
        "z": 0.845,  # table_height + half_cylinder = 0.81 + 0.035
    },
}

# Workspace region in front of the arm (table surface)
# x: 0.2–0.55, y: -0.2–0.2
WORKSPACE_X = (0.2, 0.55)
WORKSPACE_Y = (-0.2, 0.2)


def spawn_object(obj_type: str, name: str, x: float, y: float) -> bool:
    spec = OBJECT_TYPES[obj_type]
    sdf = textwrap.dedent(spec["sdf_template"]).format(
        name=name, x=x, y=y, z=spec["z"]
    )
    result = subprocess.run(
        ["gz", "service", "-s", "/world/sorting_world/create",
         "--reqtype", "gz.msgs.EntityFactory",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", f'sdf: "{sdf.strip()}"'],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        print(f"Spawned {name} at ({x:.2f}, {y:.2f})")
        return True
    else:
        print(f"Failed to spawn {name}: {result.stderr.strip()}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Spawn objects into PerceptionBOT Gazebo world")
    parser.add_argument("--count", type=int, default=4, help="Number of objects to spawn")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--clear", action="store_true", help="Print deletion commands first")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    obj_types = list(OBJECT_TYPES.keys())
    counters = {t: 0 for t in obj_types}

    spawned = []
    for i in range(args.count):
        obj_type = rng.choice(obj_types)
        x = rng.uniform(*WORKSPACE_X)
        y = rng.uniform(*WORKSPACE_Y)
        name = f"{obj_type}_{counters[obj_type]}"
        counters[obj_type] += 1
        spawned.append((obj_type, name, x, y))
        spawn_object(obj_type, name, x, y)

    print(f"\nSpawned {len(spawned)} objects:")
    for obj_type, name, x, y in spawned:
        print(f"  {name:25s}  pos=({x:.3f}, {y:.3f})")


if __name__ == "__main__":
    main()
