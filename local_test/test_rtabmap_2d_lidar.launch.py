#!/usr/bin/env python3
"""
Тестовый launch файл для rtabmap с 2D LiDAR
Запуск БЕЗ Docker, напрямую через ROS 2

Использование:
    ros2 launch local_test/test_rtabmap_2d_lidar.launch.py

Топики:
    /scan - LaserScan данные от лидара
    /odom - одометрия от icp_odometry
    /rtabmap/grid_map - карта для визуализации в RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Аргументы
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Использовать simulation time'
        ),
        
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Удалить базу данных при старте'
        ),
        
        # Отключаем Zenoh, используем стандартный DDS
        SetEnvironmentVariable('RMW_IMPLEMENTATION', ''),
        
        # ICP Odometry - одометрия по лидару
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'guess_frame_id': '',
                'wait_for_transform': 0.2,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # ICP параметры для 2D лидара
                'Icp/PointToPlane': 'false',  # 2D ICP
                'Icp/Iterations': '30',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '0.3',
                'Icp/MaxRotation': '0.78',  # ~45 градусов
                'Icp/CorrespondenceRatio': '0.4',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Odom/Strategy': '0',  # ICP odometry
                'Odom/ResetCountdown': '1',  # Автосброс если потерялся
                'Odom/ScanKeyFrameThr': '0.9',  # Создавать keyframe при 90% изменений
            }],
            remappings=[
                ('scan', '/scan'),
            ]
        ),
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'wait_for_transform': 0.5,
                'database_path': '/tmp/rtabmap_test.db',
                'approx_sync': False,
                'queue_size': 10,
                
                # RTAB-Map параметры для 2D лидара
                'RGBD/NeighborLinkRefining': 'true',  # Уточнение одометрии
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.01',  # 0.5 градуса
                'RGBD/LinearUpdate': '0.01',   # 1 см
                'RGBD/OptimizeFromGraphEnd': 'false',
                
                # Registration (ICP для лидара)
                'Reg/Strategy': '1',  # ICP
                'Reg/Force3DoF': 'true',  # 2D SLAM
                
                # ICP параметры (должны совпадать с icp_odometry)
                'Icp/PointToPlane': 'false',
                'Icp/Iterations': '30',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '0.3',
                'Icp/MaxRotation': '0.78',
                'Icp/CorrespondenceRatio': '0.4',
                'Icp/MaxCorrespondenceDistance': '0.1',
                
                # Grid параметры (карта)
                'Grid/FromDepth': 'false',
                'Grid/CellSize': '0.05',  # 5 см
                'Grid/RangeMax': '10.0',
                'Grid/RangeMin': '0.2',
                'Grid/ClusterRadius': '0.1',
                'Grid/GroundIsObstacle': 'false',
                
                # Memory
                'Mem/IncrementalMemory': 'true',  # SLAM mode
                'Mem/InitWMWithAllNodes': 'false',
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ],
            arguments=[
                '--delete_db_on_start' if LaunchConfiguration('delete_db_on_start').perform(None) == 'true' else '',
                '--udebug'  # Debug логи
            ]
        ),
        
        # Визуализация (опционально)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'wait_for_transform': 0.5,
                'approx_sync': False,
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ]
        ),
    ])
