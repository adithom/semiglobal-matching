from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_left_image_topic = DeclareLaunchArgument(
        'left_image_topic', default_value='/camera/left/image_rect',
        description='Topic for left rectified image'
    )
    declare_right_image_topic = DeclareLaunchArgument(
        'right_image_topic', default_value='/camera/right/image_rect',
        description='Topic for right rectified image'
    )
    declare_left_camera_info_topic = DeclareLaunchArgument(
        'left_camera_info_topic', default_value='/camera/left/camera_info',
        description='Camera info topic for left camera'
    )
    declare_right_camera_info_topic = DeclareLaunchArgument(
        'right_camera_info_topic', default_value='/camera/right/camera_info',
        description='Camera info topic for right camera'
    )
    declare_disparity_topic = DeclareLaunchArgument(
        'disparity_topic', default_value='/sgm_gpu/disparity',
        description='Topic to publish disparity images on'
    )
    declare_p1 = DeclareLaunchArgument(
        'p1', default_value='10',
        description='P1 penalty parameter for SGM'
    )
    declare_p2 = DeclareLaunchArgument(
        'p2', default_value='120',
        description='P2 penalty parameter for SGM'
    )
    declare_check_consistency = DeclareLaunchArgument(
        'check_consistency', default_value='true',
        description='Whether to perform left-right consistency checks'
    )

    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_camera_info_topic = LaunchConfiguration('left_camera_info_topic')
    right_camera_info_topic = LaunchConfiguration('right_camera_info_topic')
    disparity_topic = LaunchConfiguration('disparity_topic')
    p1 = LaunchConfiguration('p1')
    p2 = LaunchConfiguration('p2')
    check_consistency = LaunchConfiguration('check_consistency')

    sgm_node = Node(
        package='semiglobal-matching',
        executable='sgm_gpu_node',
        name='sgm_gpu_node',
        output='screen',
        remappings=[
            ('/camera/left/image_rect', '/input/left/image'),
            ('/camera/right/image_rect', '/input/right/image'),
            ('/camera/left/camera_info', '/input/left/camera_info'),
            ('/camera/right/camera_info', '/input/right/camera_info')
        ],
        parameters=[{
            'left_image_topic': left_image_topic,
            'right_image_topic': right_image_topic,
            'left_camera_info_topic': left_camera_info_topic,
            'right_camera_info_topic': right_camera_info_topic,
            'disparity_topic': disparity_topic,
            'p1': p1,
            'p2': p2,
            'check_consistency': check_consistency
        }]
    )

    return LaunchDescription([
        declare_left_image_topic,
        declare_right_image_topic,
        declare_left_camera_info_topic,
        declare_right_camera_info_topic,
        declare_disparity_topic,
        declare_p1,
        declare_p2,
        declare_check_consistency,
        sgm_node
    ])
