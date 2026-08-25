"""Stage 1 multi-drone bringup.

Starts the swarm-wide singletons ONCE (coordinator, safety, prompt_gateway) and
a per-drone pair (drone_controller + heartbeat_node) for each id in `drones`.

    # sim first (instances 1..3 -> /px4_1 /px4_2 /px4_3):
    #   ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 3 -w empty
    ros2 launch sim_bringup stage1_swarm.launch.py
    ros2 launch sim_bringup stage1_swarm.launch.py drones:=px4_1,px4_2

Each controller auto-arms and climbs to 3 m above its OWN spawn point (each PX4
instance has its own local origin), so the drones do not collide.
"""

import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _origin_for(drone):
    """Shared-frame origin offset for gazebo-classic sitl_multiple_run (spawn Y=3*N,
    px4_1 as the world reference): px4_k -> North = 3*(k-1), East = 0."""
    m = re.search(r'(\d+)$', drone)
    k = int(m.group(1)) if m else 1
    return f'{drone}:{3.0 * (k - 1)}:0.0'


def _spawn(context):
    drones = [d for d in LaunchConfiguration('drones').perform(context).split(',') if d]
    origins = [_origin_for(d) for d in drones]
    agentic = LaunchConfiguration('agentic').perform(context).lower() in ('1', 'true', 'yes')
    nav = LaunchConfiguration('nav').perform(context).lower() in ('1', 'true', 'yes')
    perception = LaunchConfiguration('perception').perform(context).lower() in ('1', 'true', 'yes')
    follow = LaunchConfiguration('follow').perform(context).lower() in ('1', 'true', 'yes')
    slam = LaunchConfiguration('slam').perform(context).lower() in ('1', 'true', 'yes')
    target = LaunchConfiguration('target_class').perform(context)
    camera = LaunchConfiguration('camera').perform(context).lower()
    plan_on_slam = LaunchConfiguration('plan_on_slam').perform(context).lower() in ('1', 'true', 'yes')
    search_scan = LaunchConfiguration('search').perform(context).lower() in ('1', 'true', 'yes')
    viewer = LaunchConfiguration('viewer').perform(context).lower() in ('1', 'true', 'yes')
    det_model = LaunchConfiguration('model').perform(context).strip()
    follow_alt = float(LaunchConfiguration('follow_altitude').perform(context))

    nodes = [
        # swarm-wide singletons (exactly one each)
        Node(package='formation_coordinator', executable='coordinator_node',
             name='coordinator_node', output='screen',
             parameters=[{'known_drones': drones, 'drone_origins': origins,
                          'agentic': agentic}]),
        Node(package='safety_manager', executable='safety_node',
             name='safety_node', output='screen',
             parameters=[{'drone_origins': origins}]),
        Node(package='mission_interface', executable='prompt_gateway',
             name='prompt_gateway', output='screen',
             parameters=[{'known_drones': drones}]),
    ]

    # Agentic AI loop: supervisor watches swarm health and LLM-replans on disruption.
    if agentic:
        nodes.append(Node(package='mission_interface', executable='mission_supervisor',
                          name='mission_supervisor', output='screen',
                          parameters=[{'known_drones': drones}]))

    # Autonomous navigation: one shared occupancy map + a per-drone A* planner.
    # Idle until a /px4_N/nav_goal is published, so this never disturbs mission flight.
    if nav:
        # Obstacle avoidance runs on the HORIZONTAL 2D-lidar occupancy grid (/swarm/map):
        # it sees obstacles at the drone's FLIGHT level and is altitude-robust (a downward
        # depth cam mis-reads noisy ground as obstacle from >20 m and freezes the planner).
        # Large, coarse world grid (1.0 m x 1200 = +/-600 m) so long-range goals stay on-grid.
        # The rtabmap depth grid stays for 3D MAPPING (/px4_N/map), not avoidance.
        nodes.append(Node(package='perception_node', executable='occupancy_mapper',
                          name='occupancy_mapper', output='screen',
                          parameters=[{'known_drones': drones, 'drone_origins': origins,
                                       'resolution': 1.0, 'grid_cells': 1200,
                                       'publish_rate': 1.0}]))  # big grid -> 1 Hz to bound msg load
        for d in drones:
            # map_topic default = /swarm/map (the 2D-lidar grid). plan_on_slam:=true routes the
            # drone on the SLAM grid rtabmap is building instead, so it plans against the map it
            # has actually seen; map_frame_offset applies rtabmap's map->odom correction, which
            # is what makes the planner agree with where SLAM believes the drone is.
            planner_params = {'drone_id': d, 'drone_origins': origins}
            if plan_on_slam:
                planner_params['map_topic'] = f'/{d}/map'
                planner_params['map_frame_offset'] = True
            nodes.append(Node(package='local_planner', executable='local_planner',
                              name=f'local_planner_{d}', output='screen',
                              parameters=[planner_params]))

    # per-drone control + heartbeat (unique node names)
    for d in drones:
        nodes.append(Node(package='px4_swarm_control', executable='drone_controller',
                          name=f'drone_controller_{d}', output='screen',
                          # geofence_radius matches schema.GEOFENCE_RADIUS so large-area
                          # missions aren't clamped in flight.
                          parameters=[{'drone_id': d, 'geofence_radius': 700.0}]))
        nodes.append(Node(package='swarm_communication', executable='heartbeat_node',
                          name=f'heartbeat_node_{d}', output='screen',
                          parameters=[{'drone_id': d}]))
        # In agentic mode each drone also runs an agent that self-assigns formation
        # slots via the distributed auction (and self-heals when a peer drops).
        if agentic:
            nodes.append(Node(package='formation_coordinator', executable='drone_agent',
                              name=f'drone_agent_{d}', output='screen',
                              parameters=[{'drone_id': d, 'known_drones': drones,
                                           'drone_origins': origins}]))
        # Per-drone vision detector (event-driven target detection in its own namespace).
        if perception:
            # VisDrone-YOLOv8n detector+tracker (non-blocking). target_class may be a comma
            # list of VisDrone classes (pedestrian, people, car, van, truck, bus, motor,
            # bicycle, ...). Retargetable live on /{d}/set_target.
            # FRONT CAMERA is the default sensor for detection/follow/tracking (camera:=front);
            # camera:=down uses the downward depth cam + down-cam translate control instead.
            mk = re.search(r'(\d+)$', d)
            iris = mk.group(1) if mk else '1'
            if camera == 'front':
                cam_topic = f'/iris_{iris}/front_camera/image_raw'
                cam_info = f'/iris_{iris}/front_camera/camera_info'
                mount = 'forward'
            else:
                cam_topic = f'/iris_{iris}/downward_depth_camera/image_raw'
                cam_info = f'/iris_{iris}/downward_depth_camera/camera_info'
                mount = 'down'
            det_params = {'drone_id': d, 'target_classes': target,
                          'camera_topic': cam_topic, 'camera_info_topic': cam_info,
                          'publish_world': False}
            if det_model:
                det_params['model'] = det_model
            nodes.append(Node(package='perception_node', executable='detector',
                              name=f'detector_{d}', output='screen',
                              parameters=[det_params]))
            # follow the detected target (image-space visual servoing); idle until a detection
            # arrives. search:=true -> autonomously yaw-scan for the target when unlocked, so
            # the whole detect->lock->follow loop runs with no manual intervention.
            if follow:
                # Per-drone STAGGERED follow altitude for collision safety: drone N holds
                # follow_alt + 2*(N-1) m (px4_1=base, px4_2=+2, px4_3=+4), so multiple drones
                # tracking the same target keep vertical separation instead of colliding.
                d_alt = follow_alt + 2.0 * (int(iris) - 1)
                nodes.append(Node(package='perception_node', executable='target_follow',
                                  name=f'target_follow_{d}', output='screen',
                                  parameters=[{'drone_id': d, 'follow_mode': 'image',
                                               'camera_mount': mount, 'search': search_scan,
                                               'follow_altitude': d_alt}]))
            # Live camera windows (rqt_image_view). viewer:=true opens the raw detection
            # feed + the annotated operator feed (boxes/ids) per drone. Needs a DISPLAY.
            if viewer:
                nodes.append(Node(package='rqt_image_view', executable='rqt_image_view',
                                  name=f'view_raw_{d}', arguments=[cam_topic], output='log'))
                nodes.append(Node(package='rqt_image_view', executable='rqt_image_view',
                                  name=f'view_annot_{d}', arguments=[f'/{d}/operator/image'],
                                  output='log'))

    # rtabmap RGB-D SLAM per drone, launched from the MAIN launch (no separate command).
    # NOTE: this provides the map + rviz; nav still uses the lidar occupancy_mapper because
    # the planner works in a shared world frame while rtabmap maps per-drone px4_N/map
    # frames. Wiring nav onto rtabmap needs frame reconciliation (see audit) — not done here.
    if slam:
        slam_viz = LaunchConfiguration('slam_viz').perform(context)
        share = get_package_share_directory('sim_bringup')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f'{share}/launch/rtabmap_slam.launch.py'),
            launch_arguments={'drones': ','.join(drones),
                              'rviz': slam_viz}.items()))   # slam_viz:=true opens rtabmap GUI
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drones', default_value='px4_1,px4_2,px4_3'),
        DeclareLaunchArgument('agentic', default_value='true',
                              description='Decentralised slot auction via per-drone agents'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='Online occupancy mapping + A* autonomous navigation'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='Per-drone YOLO target detector'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='Per-drone target-follow (visual servoing)'),
        DeclareLaunchArgument('target_class', default_value='pedestrian,people',
                              description='Object class the detectors look for'),
        DeclareLaunchArgument('camera', default_value='front',
                              description="Detection/follow sensor: 'front' (forward servo) or 'down'"),
        DeclareLaunchArgument('search', default_value='true',
                              description='Follower autonomously yaw-scans for the target when unlocked'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='Open per-drone rqt_image_view windows (raw + annotated)'),
        DeclareLaunchArgument('follow_altitude', default_value='3.0',
                              description='Altitude (m) the follower holds while pursuing a target'),
        DeclareLaunchArgument('model', default_value='',
                              description='Detector YOLO weights path; empty = detector default '
                                          '(VisDrone). Use models/yolov8n.pt (COCO) for ground '
                                          'person/car views — VisDrone is aerial-trained.'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Launch per-drone rtabmap 3D-LiDAR + RGB-D SLAM'),
        DeclareLaunchArgument('plan_on_slam', default_value='false',
                              description='Plan on the rtabmap SLAM grid instead of the lidar grid'),
        DeclareLaunchArgument('slam_viz', default_value='false',
                              description='Open the rtabmap GUI (map + graph viewer)'),
        OpaqueFunction(function=_spawn),
    ])
