"""rtabmap 3D-LiDAR + RGB-D SLAM per drone, namespaced, optional GUI.

Uses ONLY rtabmap_ros (rtabmap + PX4-EKF odom via px4_odom_tf) + static sensor TFs — no
custom mapper. One rtabmap instance PER drone, each in its own namespace with its own TF
frames (px4_N/map -> px4_N/odom -> px4_N/base_link -> camera/lidar), so the swarm's maps
never collide. Run it for the whole swarm by listing every drone in `drones:=`.

Each drone fuses two onboard sensors: the downward RGB-D camera (visual features for loop
closure / relocalisation) and the sweeping 3D LiDAR (/iris_N/scan_3d), which the occupancy
grid is built from — the LiDAR sees vertical structure (shelves, walls) at flight level
that a top-down camera misses, so the nav map has real obstacles.

Needs a feature-rich scene (empty world = nothing to map; use warehouse/ksql_airport).
Frames: base_link -> camera from the iris SDF (xyz 0 0 -0.05, pitch 90deg down) + the ROS
optical rotation; base_link -> lidar_3d_link from the SDF (xyz 0 0 0.15).

    ros2 launch sim_bringup rtabmap_slam.launch.py drones:=px4_1,px4_2,px4_3
    ros2 launch sim_bringup rtabmap_slam.launch.py drones:=px4_1 rviz:=true   # open GUI
"""

import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# SLAM fuses the drone's 3D LiDAR + downward RGB-D camera: the camera gives visual
# features (loop closure / relocalisation), the LiDAR gives the geometry the occupancy
# grid is built from. A sweeping 3D LiDAR sees vertical structure (shelves, walls) at
# flight level directly, which a top-down depth camera barely captures — so the nav map
# has real obstacles and the swarm can plan around them.
#
# Grid args:
#   -d / --delete_db_on_start: start each run with a FRESH map DB, else rtabmap reopens a
#     stale ~/.ros/rtabmap.db and can fatally abort ("another row available").
#   Grid/FromDepth=false: build the occupancy grid from the LiDAR scan cloud, not the depth
#     image (the LiDAR is the better obstacle sensor here).
#   Grid/RayTracing=true: carve free space along each LiDAR ray so the map is clean, not a
#     scatter of hit points.
#   NormalsSegmentation=false + MapFrameProjection=true: classify ground vs obstacle by
#     HEIGHT IN THE MAP FRAME (floor ~0 m). Otherwise, from altitude, floor and flat tops
#     both read as "ground" and nothing becomes an obstacle.
#   MaxGroundHeight=0.3: anything taller than 0.3 m (shelves, pallets, boxes) is an obstacle.
#   CellSize=0.2: 0.2 m grid (vs 0.05 m default) — ample for drone nav, ~16x cheaper for A*.
# MaxGroundHeight=3.0 (was 0.3): at higher altitude the depth camera's range noise
# spreads ground points in Z, and a tight 0.3 m ground threshold flips that noisy
# ground into "obstacle" (map went ~59% occupied at 30 m and boxed the drones in). A
# 3 m band keeps the ground + low clutter as FREE; only real structures >3 m are obstacles.
_GRID_ARGS = ('-d --Grid/RangeMax 40 --Grid/RangeMin 0.3 --Grid/CellSize 0.5 '
              '--Grid/FromDepth true --Grid/RayTracing true '
              '--Grid/NormalsSegmentation false --Grid/MapFrameProjection true '
              '--Grid/MaxGroundHeight 3.0 --Grid/MaxObstacleHeight 15')


def _setup(context):
    drones = [d for d in LaunchConfiguration('drones').perform(context).split(',') if d]
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes')
    rtabmap_launch = get_package_share_directory('rtabmap_launch')

    nodes = []
    for drone in drones:
        k = re.search(r'(\d+)$', drone)
        k = k.group(1) if k else '1'                 # iris/px4 instance index
        cam = f'iris_{k}/downward_depth_camera'
        cam_link = f'iris_{k}/downward_depth_camera_link'
        lidar_link = f'iris_{k}/lidar_3d_link'        # frame of /iris_k/scan_3d cloud
        base, odom, mount, mapf = (f'{drone}/base_link', f'{drone}/odom',
                                   f'{drone}/cam_mount', f'{drone}/map')
        # Each drone's whole SLAM sub-graph goes in a scoped GroupAction. rtabmap.launch.py
        # declares LaunchConfigurations (rgb_topic, ...); included flat 3x they collide and
        # every drone's rtabmap ends up on iris_1's camera. A GroupAction scopes them so
        # each include keeps its OWN topics/frames.
        nodes.append(GroupAction([
            # PX4 EKF pose -> odom + odom->base_link TF, with per-drone frames.
            Node(package='perception_node', executable='px4_odom_tf',
                 name=f'px4_odom_tf_{drone}',
                 parameters=[{'drone_id': drone, 'odom_frame': odom, 'base_frame': base,
                              'use_sim_time': True}]),
            # base_link -> mount (physical, from SDF) -> camera optical frame.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'cam_mount_{drone}',
                 arguments=['0', '0', '-0.05', '0', '1.5708', '0', base, mount]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'cam_optical_{drone}',
                 arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', mount, cam_link]),
            # base_link -> 3D LiDAR (SDF: lidar_3d_joint at 0 0 0.15, no rotation), so
            # rtabmap can place the /iris_k/scan_3d cloud in the map.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'lidar3d_{drone}',
                 arguments=['0', '0', '0.15', '0', '0', '0', base, lidar_link]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f'{rtabmap_launch}/launch/rtabmap.launch.py'),
                launch_arguments={
                    'namespace':          drone,         # /px4_N/rtabmap ...
                    'rgb_topic':          f'/{cam}/image_raw',
                    'depth_topic':        f'/{cam}/depth/image_raw',
                    'camera_info_topic':  f'/{cam}/depth/camera_info',
                    # 3D LiDAR fused in: geometry for the occupancy grid + scan matching.
                    'subscribe_scan_cloud': 'true',
                    'scan_cloud_topic':   f'/iris_{k}/scan_3d',
                    'frame_id':           base,
                    'odom_frame_id':      odom,
                    'map_frame_id':       mapf,
                    'odom_topic':         f'/{drone}/odom',
                    # per-drone DB, else 3 rtabmaps fight over ~/.ros/rtabmap.db ("locked").
                    'database_path':      f'/tmp/rtabmap_{drone}.db',
                    'qos':                '2',
                    'approx_sync':        'true',
                    # TF from PX4 odometry arrives behind the camera frames on a loaded
                    # machine. rtabmap's default wait is short enough that the lookup fails
                    # ("Lookup would require extrapolation") and the frame is dropped, so the
                    # map stays sparse. Half a second covers the observed lag and only costs
                    # latency on frames that would otherwise be discarded.
                    'wait_for_transform':  '0.5',
                    'visual_odometry':    'false',       # PX4 EKF odom
                    'rtabmap_viz':        'true' if rviz else 'false',
                    'rviz':               'false',
                    'use_sim_time':       'true',
                    'args':               _GRID_ARGS,
                }.items()),
        ]))
    if rviz:                                            # one shared viewer (add displays per drone)
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz', output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drones', default_value='px4_1',
                              description='comma list of drones to run SLAM on'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='launch one shared rviz2'),
        OpaqueFunction(function=_setup),
    ])
