from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	file_arg = DeclareLaunchArgument('file', default_value='/etc/xela/xServ.ini')
	port_arg = DeclareLaunchArgument('port', default_value='5000')
	ip_arg   = DeclareLaunchArgument('ip', default_value='127.0.0.1')
	d_arg    = DeclareLaunchArgument('d', default_value='0')

	file_cfg = LaunchConfiguration('file')
	port_cfg = LaunchConfiguration('port')
	ip_cfg   = LaunchConfiguration('ip')
	d_cfg    = LaunchConfiguration('d')

	return LaunchDescription([
		file_arg, port_arg, ip_arg, d_arg,

		# Launch system-installed xela_server in urxvt
		ExecuteProcess(
			cmd=[
				'xela_server',
				'-f', file_cfg,
				'-i', ip_cfg,
				'-p', port_cfg
			],
			output='screen'
		),

		# Delay xela_service launch by 5 seconds
		TimerAction(
			period=5.0,
			actions=[
				Node(
					package='xela_server_ros2',
					executable='xela_service',
					name='xela_service',
					output='screen',
					parameters=[{
						'port': port_cfg,
						'ip': ip_cfg,
						'd': d_cfg
					}]
				)
			]
		)
	])

