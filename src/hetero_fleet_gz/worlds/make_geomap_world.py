#!/usr/bin/env python3
"""Build the fleet world on top of the scanned map.

The source model, vendored at worlds/source/model.sdf (originally the desktop
`~/Desktop/model.sdf`, model `geomap_final_noroof`) is a *viewer* model: the coloured
scan with a triangle-mesh collider. It is missing everything a fleet
simulation needs, and one of the things it does have (mesh collision under the
default physics engine) does not survive contact with the rest of this
workspace. This script generates the fleet world from scratch instead of
editing the user's file, so the original stays untouched and the derivation is
repeatable:

  1. Physics with BULLET. The scan's collision geometry is a triangle mesh;
     DART/ODE segfault on mesh collisions in this Gazebo (that is exactly why
     the user's own world file pins Bullet - see its comment).

  2. Sensors + scene systems, which the source world lacks. Without the
     sensors system the GPU lidars on the robots never publish a scan and Nav2
     has nothing to build a costmap from.

  3. A ground plane at the height the robots drive on. The map model is
     levelled so its floor sits at z=0 (tools/prepare_map_mesh.py) and the floor
     faces were removed from its collision mesh, so this plane *is* the floor.

  4. Pickup / dropoff / charger markers, at coordinates verified against the
     occupancy grid by tools/pick_zones.py.

Usage:
    python3 make_geomap_world.py [output.sdf]
"""

import sys
from pathlib import Path

MODEL = "geomap_final_noroof"
SDF_VERSION = "1.11"

# Vertical placement. Three numbers, all in metres:
#
#   MAP_Z     the scan model is lifted by this much so its floor (which
#             prepare_map_mesh.py levels to +0.027 m) sits clearly above the
#             world origin instead of straddling it.
#   GROUND_Z  the (invisible) plane the robots actually drive on. Keep it just
#             under the floor's surface: wheels then roll ON the blue floor.
#   GROUND_VISUAL_DZ  how far DOWN the grey 300x300 plane is drawn. The scan's
#             floor is bumped by +-0.1 m, so a grey plane level with it wins the
#             depth test in half the pixels and hides the blue floor completely
#             (exactly the "can't see the blue floor" symptom). Sinking the
#             *visual* only means the physics still uses GROUND_Z while the
#             scan's floor is never covered by anything grey.
MAP_Z = 0.10
# floor surface lands at MAP_Z + 0.027, so put the plane ~1 cm under it
GROUND_Z = MAP_Z + 0.017
GROUND_VISUAL_DZ = -0.30

# (name, x, y, sx, sy, rgba) - coordinates from tools/pick_zones.py, which
# verifies every one of them against the occupancy grid for the whole fleet's
# clearance requirement. They all sit in the same connected aisle, so a task
# between any pickup and any dropoff is plannable.
ZONES = [
    # --- work zones: the eastern half of the ring -----------------------
    # Six pads, alternating pickup / dropoff, strung round the eastern half of
    # the ring: west-to-east along the +4.2 m arm and back west along the south
    # corridor. Consecutive pads are 1.9-3.7 m apart, which is loose on purpose
    # - a pickup and its dropoff used to sit 2.2 m apart, and with four robots
    # working the same corner of a 1.8 m-wide corridor that was where every jam
    # in the measured runs happened.
    #
    # Nothing is placed in the pinch at x ~ 5.5-6.8 on the south corridor
    # (clearance drops to 0.6 m there - the narrowest point of the whole ring),
    # and nothing is placed immediately either side of it: a pad there leaves
    # only ~0.6 m of lane past a robot parked on it, which is what the measured
    # run showed when omni_1 sat for 45 s at (7.0, -0.45) - 0.4 m west of
    # dropoff_A, waiting for the pinch to clear. dropoff_A therefore sits at
    # x = 8.1, where the lane past it is 1.1 m.
    #
    # Nothing goes in the EAST CONNECTOR either (x ~ 11.6-12.4, y -0.3..4.2).
    # It is the ring's only north-south link at that end, and dropoff_C sat in
    # the middle of it at (12.05, 2.40). On a measured run that one choice
    # accounted for 25 of the fleet's 42 stall events: a robot working there
    # closes the corridor, every north-south trip has to meet it head on, and
    # the meeting turns into a standoff. It now sits on the north arm at
    # (11.0, 4.22), which leaves the connector clear.
    ("pickup_A",  10.60,  -0.43, 0.6, 0.6, "0.10 0.85 0.20 0.55"),
    ("pickup_B",   1.70,   4.22, 0.6, 0.6, "0.10 0.85 0.20 0.55"),
    ("pickup_C",   9.10,   4.22, 0.6, 0.6, "0.10 0.85 0.20 0.55"),
    ("dropoff_A",   8.10,  -0.43, 0.6, 0.6, "0.15 0.35 0.90 0.55"),
    ("dropoff_B",  5.40,   4.22, 0.6, 0.6, "0.15 0.35 0.90 0.55"),
    ("dropoff_C", 11.00,   4.22, 0.6, 0.6, "0.15 0.35 0.90 0.55"),
    # Additional pads spread transport work along both arms of the ring.
    ("pickup_D", -2.00, 4.22, 0.6, 0.6, "0.10 0.85 0.20 0.55"),
    ("pickup_E", 5.80, -0.20, 0.6, 0.6, "0.10 0.85 0.20 0.55"),
    ("dropoff_D", 7.00, 4.22, 0.6, 0.6, "0.15 0.35 0.90 0.55"),
    ("dropoff_E", 0.00, 4.22, 0.6, 0.6, "0.15 0.35 0.90 0.55"),
    # --- the charging station: a row against the south wall --------------
    # One charger per robot: every vehicle's spawn pose IS its charger (see
    # robot_profiles.yaml) and it drives back here when the fleet runs out of
    # work, so a fleet of four needs four bays.
    #
    # They are parked nose-east in a 1.5 m row against the SOUTH WALL of the
    # ring's south corridor, just west of the pinch at x ~ 5.5-6.8. Three
    # things decided the spot:
    #
    #   * the corridor there is 2.2-2.4 m wide, so the row still leaves
    #     >= 1.0 m of travel lane north of a parked robot. Every bay was
    #     checked for exactly that in the occupancy grid (see below) - a
    #     charging row that blocks the ring is worse than no charging row.
    #   * the bays are 1.5 m apart, i.e. four times the widest robot's width,
    #     so each one can pull straight out east or west without shunting its
    #     neighbour, unlike the nose-to-tail arrangement this replaced.
    #   * east of the pinch there is only 3.2 m of wall clear enough for a
    #     row, which fits three robots, not four.
    #
    # The y values follow the wall rather than forming a perfectly straight
    # line, because the wall does not: it steps 0.2 m between x = 1.2 and 2.7.
    # tools/verify_scene.py checks every one of them (0.70-0.72 m clearance;
    # the widest robot needs 0.53 m).
    ("charger_A",  -0.30,  -0.25, 0.5, 0.5, "0.95 0.80 0.10 0.55"),
    ("charger_B", 1.20, -0.80, 0.50, 0.50, "0.25 0.90 0.65 0.55"),
    ("charger_C",   2.70,  -0.45, 0.5, 0.5, "0.95 0.80 0.10 0.55"),
    ("charger_D",   4.60,  -0.45, 0.5, 0.5, "0.95 0.80 0.10 0.55"),
]

HEAD = f"""<?xml version='1.0'?>
<!-- generated by make_geomap_world.py - do not edit by hand -->
<sdf version='{SDF_VERSION}'>
  <world name='geomap_fleet'>
    <!-- DART, not Bullet.

         The world used to pin Bullet because DART segfaults on the map's
         209k-triangle collision mesh. But Bullet-Featherstone cannot rotate a
         wheeled base on this map at all: the drive wheels counter-rotate at
         exactly the commanded rate while the body yaw stays at zero, so every
         Nav2 goal that needs a turn dies with "Failed to make progress". The
         same thing happens to Gazebo's own reference diff-drive demo model
         when Bullet is forced, and the same robots turn correctly under DART -
         the engine is the variable, not the robot model.

         The mesh collision is gone: tools/make_collision_boxes.py now turns
         the Nav2 occupancy grid into ~1.6k axis-aligned box colliders, which
         DART handles natively (and which match the costmap cell for cell).
         See that script's docstring for the full story.

         If you ever regenerate the map, keep the order:
           prepare_map_mesh.py -> make_nav2_map.py -> make_collision_boxes.py
           -> make_geomap_world.py -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-dartsim-plugin</filename></engine>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- Missing from the viewer world. No sensors system = no lidar = Nav2
         has no costmap and every robot just sits there.
         render_engine: ogre2 draws mesh materials white in this install, so
         the coloured scan must be rendered by the ogre (1) engine; see
         gz_sim.launch.py, which passes the matching render-engine argument.
         (Written without the leading dashes on purpose: a double hyphen is
         illegal inside an XML comment and makes the file unparseable by
         xmllint / ElementTree.) -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- 1 ms steps keep the scaled G1 ankle torque controller stable. -->
    <physics name='g1_1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <!-- Brighter ambient light keeps the scanned colours readable in shade. -->
    <scene>
      <ambient>0.80 0.80 0.80 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- The physical driving surface coincides with the model flat_floor
         visual at world z=0.117. The large grey visual is sunk out of sight. -->
    <model name='ground_plane'>
      <static>true</static>
      <pose>0 0 {GROUND_Z} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <bounce><restitution_coefficient>0.0</restitution_coefficient></bounce>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <pose>0 0 {GROUND_VISUAL_DZ} 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <material>
            <ambient>0.35 0.35 0.36 1</ambient>
            <diffuse>0.35 0.35 0.36 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- The scan: coloured visual mesh + the same surface without its floor as
         collision. Both are levelled so the floor is at z=0. -->
    <include>
      <uri>model://{MODEL}</uri>
      <name>{MODEL}</name>
      <pose>0 0 {MAP_Z} 0 0 0</pose>
    </include>
"""


def zone_sdf() -> str:
    chunks = []
    for name, x, y, sx, sy, rgba in ZONES:
        chunks.append(f"""    <model name='zone_{name}'>
      <static>true</static>
      <pose>{x} {y} {GROUND_Z + 0.03:.3f} 0 0 0</pose>
      <link name='pad'>
        <visual name='pad_visual'>
          <pose>0 0 0.01 0 0 0</pose>
          <geometry><box><size>{sx} {sy} 0.02</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        </visual>
        <visual name='rim_visual'>
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>{sx} {sy} 0.08</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        </visual>
      </link>
    </model>
""")
    return "".join(chunks)


def main() -> int:
    here = Path(__file__).resolve().parent
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else here / "geomap_fleet.sdf"
    out.write_text(HEAD + zone_sdf() + "  </world>\n</sdf>\n")
    print(f"wrote {out}")
    print(f"  map model   : model://{MODEL} (visual + box colliders from the "
          f"occupancy grid)")
    print(f"  scan model   : z = {MAP_Z}  (flat floor lands at {GROUND_Z:.3f} m)")
    print(f"  driving plane: z = {GROUND_Z} (visual sunk {GROUND_VISUAL_DZ})")
    print(f"  zones       : {len(ZONES)} markers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
