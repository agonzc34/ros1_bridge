from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from yaml.loader import SafeLoader
import os


def open_yaml_file():
	with open("/root/conf.yaml") as f:
		data = yaml.load(f, Loader=SafeLoader)
		return data

def generate_launch_description():
	parser = {"twist":"simple_bridge_2_to_1_twist", 
	"odom":"simple_bridge_1_to_2_odom",
	"tf": "simple_bridge_1_to_2_tf",
	"image": "simple_bridge_1_to_2_image",
	"scan": "simple_bridge_1_to_2_scan",
	"imu": "simple_bridge_1_to_2_imu",
	"range": "simple_bridge_1_to_2_sonar",
	"compressed": "simple_bridge_1_to_2_compressed",
	"point_cloud2": "simple_bridge_1_to_2_point_cloud",
	"camera_info": "simple_bridge_1_to_2_camera_info",
    "log": "simple_bridge_1_to_2_log",
	"joint_state": "simple_bridge_1_to_2_jointState",
	"joint_trajectory": "simple_bridge_2_to_1_joint_trajectory",
	"joint_trajectory_controller_state": "simple_bridge_1_to_2_joint_trajectory_controller_state",
	}

	
	data = open_yaml_file()

	counter = 0
	ld = LaunchDescription()
	
	
	for bridge in data["bridges"]:
	
		bridge_node = Node(
			package ="ros1_bridge",
			executable = parser[bridge["msg_type"]],
			name = f"ros1_bridge{counter}",
			parameters=[
                {'topic_name': bridge["topic_name"]}
            ]
		)
		counter+=1
		ld.add_action(bridge_node)
		
	return ld
