#!/usr/bin/env python3
"""Generate a Gazebo world SDF with the physical obstacles for an ADMM arena, so the
distributed fleet can be seen weaving through the plum poles / door in sim.

The ADMM already KNOWS these obstacles (admm_core/fleet_config.cpp arenas(): CBF + A*);
this just puts matching physical bodies in Gazebo. legged_gazebo/worlds is a read-only
submodule, so the output goes to legged_admm_fleet/worlds/<arena>.sdf and gazebo.launch.py
must be pointed at it (scripts/arena_run.sh passes the full path).

KEEP OBSTACLE DATA IN SYNC with admm_core/fleet_config.cpp arenas(). Values copied verbatim.
Usage: gen_arena_world.py plum|door  ->  writes ../worlds/<arena>.sdf
"""
import sys
from pathlib import Path

# --- mirror of fleet_config.cpp arenas() (circles: (x,y,radius); rect walls: (cx,cy,sx,sy)) ---
ARENAS = {
    "empty": {"circles": [], "rects": [], "cam": None},  # ground+sun only; single gazebo_package
    "plum": {
        # Positions match fleet_config.cpp plum (7-row quincunx, 17 pegs for Vision60).
        # Physical peg radius is the ROS1 value 0.20 (NOT the 0.30 CBF radius, which is
        # planner inflation, not the real body); gives the trot true footprint clearance.
        "circles": [(x, y, 0.20) for (x, y) in [
            (4.76, 1.54), (4.76, -1.54), (6.58, 0.0), (6.58, 2.94), (6.58, -2.94),
            (8.40, 1.54), (8.40, -1.54), (10.22, 0.0), (10.22, 2.94), (10.22, -2.94),
            (12.04, 1.54), (12.04, -1.54), (13.86, 0.0), (13.86, 2.94), (13.86, -2.94),
            (15.68, 1.54), (15.68, -1.54)]],
        "rects": [],
        "cam": (10.2, -13, 15, 0, 0.85, 1.5708),  # overhead of the ~x[4.8,15.7] field
    },
    "plum_dense": {  # 0.958x-scaled plum, min gap 2.20 m (matches fleet_config plum_dense)
        "circles": [(x, y, 0.20) for (x, y) in [
            (4.56, 1.48), (4.56, -1.48), (6.31, 0.0), (6.31, 2.82), (6.31, -2.82),
            (8.05, 1.48), (8.05, -1.48), (9.79, 0.0), (9.79, 2.82), (9.79, -2.82),
            (11.54, 1.48), (11.54, -1.48), (13.28, 0.0), (13.28, 2.82), (13.28, -2.82),
            (15.02, 1.48), (15.02, -1.48)]],
        "rects": [],
        "cam": (9.8, -12, 14, 0, 0.85, 1.5708),
    },
    "door": {  # A1-native door scaled 1.4x for Vision60 (matches fleet_config.cpp arenas() door)
        "circles": (
            [(x, y, 0.30) for (x, y) in [(10.5, 3.5), (11.9, -2.1), (12.6, 4.9), (9.8, -4.2), (11.2, 0.7)]] +
            [(x, y, 0.15) for (x, y) in [(8.4, 1.75), (8.4, 2.45), (8.4, -1.75), (8.4, -2.45)]]),
        "rects": [  # (center_x, center_y, size_x, size_y)
            (8.4, 4.375, 0.21, 5.25), (8.4, -4.375, 0.21, 5.25),
            (14.0, 0.0, 0.21, 14.0), (11.2, 7.0, 5.6, 0.21), (11.2, -7.0, 5.6, 0.21)],
        "cam": (7.0, -10, 13, 0, 0.85, 1.5708),
    },
}
H = 1.0  # obstacle height [m]


def cylinder(i, x, y, r):
    return f"""    <model name="obs_cyl_{i}"><static>true</static>
      <pose>{x} {y} {H/2} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><cylinder><radius>{r}</radius><length>{H}</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{H}</length></cylinder></geometry>
      <material><ambient>0.7 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual></link></model>"""


def box(i, cx, cy, sx, sy):
    return f"""    <model name="wall_{i}"><static>true</static>
      <pose>{cx} {cy} {H/2} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>{sx} {sy} {H}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx} {sy} {H}</size></box></geometry>
      <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material></visual></link></model>"""


def camera(pose):
    # Overhead recording camera; needs the Sensors system plugin (already in the header).
    # arena_run.sh RECORD=1 bridges the arena_cam/image topic -- regen without this drops it.
    px, py, pz, r, p, yaw = pose
    return f"""    <model name="arena_cam"><static>true</static>
      <pose>{px} {py} {pz} {r} {p} {yaw}</pose>
      <link name="link">
        <sensor name="arena_cam" type="camera">
          <camera>
            <horizontal_fov>1.1</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.1</near><far>200</far></clip>
          </camera>
          <always_on>1</always_on><update_rate>30</update_rate>
          <topic>arena_cam/image</topic>
        </sensor>
      </link>
    </model>"""


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ARENAS:
        sys.exit(f"usage: {sys.argv[0]} {'|'.join(ARENAS)}")
    name = sys.argv[1]
    a = ARENAS[name]
    bodies = [cylinder(i, x, y, r) for i, (x, y, r) in enumerate(a["circles"])]
    bodies += [box(i, *r) for i, r in enumerate(a["rects"])]
    if a.get("cam"):
        bodies.append(camera(a["cam"]))
    # World header copied from legged_gazebo/worlds/empty.sdf (sdf 1.9, dart, same plugins)
    # so it loads identically in the container's gz; obstacle bodies injected before </world>.
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <dart>
        <solver><solver_type>pgs</solver_type></solver>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>
{chr(10).join(bodies)}
  </world>
</sdf>
"""
    out = Path(__file__).resolve().parent.parent / "worlds" / f"{name}.sdf"
    out.parent.mkdir(exist_ok=True)
    out.write_text(sdf)
    print(f"wrote {out} ({len(a['circles'])} cylinders, {len(a['rects'])} walls)")


if __name__ == "__main__":
    main()
