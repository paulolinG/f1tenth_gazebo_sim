import math

# Track parameters
CAR_WIDTH = 0.2032
TRACK_WIDTH = CAR_WIDTH * 7           
STRAIGHT_LENGTH = 8.0         
SEMICIRCLE_RADIUS = 1.2            
WALL_HEIGHT = 0.25         
WALL_THICKNESS = 0.05
ARC_SEGMENTS = 24       

INNER_R = SEMICIRCLE_RADIUS - TRACK_WIDTH / 2.0
OUTER_R = SEMICIRCLE_RADIUS + TRACK_WIDTH / 2.0


def wall_model(name, x, y, z, yaw, length, width=WALL_THICKNESS, height=WALL_HEIGHT):
    """Return an SDF model string for a single wall segment."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{length:.6f} {width:.6f} {height:.6f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{length:.6f} {width:.6f} {height:.6f}</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def generate_walls():
    walls = []
    idx = 0
    half_straight = STRAIGHT_LENGTH / 2.0
    z = WALL_HEIGHT / 2.0

    # Each straight runs along the X axis, offset in Y by the track half-width
    for side_label, y_offset in [("outer", OUTER_R), ("inner", INNER_R)]:
        # Top straight (positive Y side)
        walls.append(wall_model(
            f"wall_top_{side_label}", 0, y_offset, z, 0, STRAIGHT_LENGTH
        ))
        # Bottom straight (negative Y side)
        walls.append(wall_model(
            f"wall_bot_{side_label}", 0, -y_offset, z, 0, STRAIGHT_LENGTH
        ))

    # Right semicircle (x = +half_straight, curves from +Y to -Y)
    # Left semicircle (x = -half_straight, curves from -Y to +Y)

    for arc_label, cx in [("right", half_straight), ("left", -half_straight)]:
        for radius_label, R in [("outer", OUTER_R), ("inner", INNER_R)]:
            for i in range(ARC_SEGMENTS):
                if arc_label == "right":
                    a0 = math.pi / 2.0 - i * math.pi / ARC_SEGMENTS
                    a1 = math.pi / 2.0 - (i + 1) * math.pi / ARC_SEGMENTS
                else:
                    a0 = -math.pi / 2.0 - i * math.pi / ARC_SEGMENTS
                    a1 = -math.pi / 2.0 - (i + 1) * math.pi / ARC_SEGMENTS

                x0 = cx + R * math.cos(a0)
                y0 = R * math.sin(a0)
                x1 = cx + R * math.cos(a1)
                y1 = R * math.sin(a1)

                mx = (x0 + x1) / 2.0
                my = (y0 + y1) / 2.0
                seg_len = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                yaw = math.atan2(y1 - y0, x1 - x0)

                walls.append(wall_model(
                    f"wall_{arc_label}_{radius_label}_{idx}",
                    mx, my, z, yaw, seg_len + 0.01  # slight overlap to avoid gaps
                ))
                idx += 1

    return "\n".join(walls)


def generate_ground():
    """Large white ground plane for contrast with black walls."""
    size = max(STRAIGHT_LENGTH + 2 * OUTER_R + 2, 2 * OUTER_R + 2)
    return f"""
    <model name="track_ground">
      <static>true</static>
      <pose>0 0 0.001 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size:.1f} {size:.1f} 0.001</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size:.1f} {size:.1f} 0.001</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world():
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="button_track">

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane (Gazebo default for physics) -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Track ground surface -->
    {generate_ground()}

    <!-- ═══════════════ Track walls ═══════════════ -->
    {generate_walls()}

  </world>
</sdf>
"""


if __name__ == "__main__":
    world_content = generate_world()
    output_path = "button_track.world"
    with open(output_path, "w") as f:
        f.write(world_content)

    print(f"Generated: {output_path}")
    print(f"Track width:       {TRACK_WIDTH:.3f} m ({TRACK_WIDTH/CAR_WIDTH:.1f} car widths)")
    print(f"Straight length:   {STRAIGHT_LENGTH:.1f} m")
    print(f"Semicircle radius: {SEMICIRCLE_RADIUS:.1f} m (centerline)")
    print(f"Inner radius:      {INNER_R:.3f} m")
    print(f"Outer radius:      {OUTER_R:.3f} m")
    print(f"Arc segments:      {ARC_SEGMENTS} per semicircle")
    print()
    print("Usage in launch file:")
    print("  gazebo = IncludeLaunchDescription(")
    print("      PythonLaunchDescriptionSource([os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')]),")
    print("      launch_arguments={'world': '<path>/button_track.world'}.items()")
    print("  )")