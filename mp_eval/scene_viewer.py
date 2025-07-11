#!/usr/bin/env python3

import subprocess
import logging
from typing import Dict, List, Optional, Any
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from mp_eval.classes.workload import WorkloadConfig  # plus other imports as needed
from mp_eval.classes.scene_generator import SceneGenerator
# import os
# import time
# from threading import Thread
import signal
import traceback
import time


import rclpy
from rclpy.node import Node

class SceneViewer(Node):
	def __init__(self, node_name='scene_viewer'):		
		super().__init__(node_name)
		self.logger = self.get_logger()

		# ros parameters
		self.declare_parameter('workload', '.')
		self.workload = self.get_parameter('workload').value

		self.pkg_dir = Path(get_package_share_directory('percept'))

		self._setup()


	def _setup(self):
		# Store node specifications by node name.
		self.active_nodes: Dict[str, Dict[str, Any]] = {}
		# Store subprocess.Popen objects by node name.
		self.active_processes: Dict[str, subprocess.Popen] = {}

		# load workload config
		self.config = WorkloadConfig.from_yaml(self.workload).percept_config
		
	def _add_node(
		self,
		package_name: str,
		node_executable: str,
		node_name: str,
		parameters: Optional[Dict[str, Any]] = None,
		remappings: Optional[List[tuple]] = None,
		namespace: str = "",
		arguments: Optional[List[str]] = None
	) -> Dict[str, Any]:
		"""
		Create a new node specification.
		
		Args:
			package_name: Name of the ROS2 package.
			node_executable: Name of the node executable.
			node_name: Name for this node instance.
			parameters: Dictionary of parameters to pass.
			remappings: List of remapping tuples (from, to).
			namespace: Namespace for the node.
			arguments: Additional command line arguments.
		"""
		node_spec = {
			"package": package_name,
			"executable": node_executable,
			"name": node_name,
			"parameters": parameters,
			"remappings": remappings or [],
			"namespace": namespace,
			"arguments": arguments or []
		}
		self.active_nodes[node_name] = node_spec
		return node_spec
    
	def _node_to_command(self, node_spec: Dict[str, Any]) -> List[str]:
		"""
		Convert a node specification into a command list.
		This command uses the typical ROS2 CLI format:
			ros2 run <package> <executable> [arguments] --ros-args 
				-r __node:=<node_name> 
				[-r __ns:=<namespace>] 
				[-r <remap_from>:={<remap_to>}]... 
				[-p <param_key>:={<param_value>}]...
		"""
		cmd = ["ros2", "run", node_spec["package"], node_spec["executable"]]
		if node_spec["arguments"]:
			cmd.extend(node_spec["arguments"])
		# Begin ROS2-specific arguments.
		cmd.append("--ros-args")
		# Set the node name.
		cmd.extend(["-r", f"__node:={node_spec['name']}"])
		# Set the namespace if provided.
		if node_spec["namespace"]:
			cmd.extend(["-r", f"__ns:={node_spec['namespace']}"])
		# Add any remappings.
		for from_topic, to_topic in node_spec["remappings"]:
			cmd.extend(["-r", f"{from_topic}:={to_topic}"])
		# Add parameters.
		if node_spec["parameters"]:
			for key, value in node_spec["parameters"].items():
				cmd.extend(["-p", f"{key}:={value}"])
		return cmd

	def _setup_scene_nodes(self):
		scene_type = self.config.scene_config.scene_type
		publish_once = self.config.scene_config.scene_params.publish_once
		if scene_type == "static":
			scene_path = self.config.scene_config.scene_params.static_scene_path
			self._add_node(
				package_name="percept",
				node_executable="scene_loader.py",
				node_name="scene_loader",
				parameters={
					'obstacles_config_path': str(self.pkg_dir / scene_path),
					'publish_once': publish_once
				}
			)
		elif scene_type == "generated":
			scene_path = 'assets/benchmark_scenes/auto_generated_scene.yaml'
			self._add_node(
				package_name="percept",
				node_executable="scene_loader.py",
				node_name="scene_loader",
				parameters={
					'obstacles_config_path': str(self.pkg_dir / scene_path),
					'publish_once': publish_once
				}
			)
		else:
			raise ValueError(f"Scene type {self.config.scene_config.scene_type} not supported")
		

	def _setup_rviz_node(self):
		if self.config.rviz_config.show_rviz:
			rviz_config_path = self.config.rviz_config.rviz_config_path
			self._add_node(
				package_name="rviz2",
				node_executable="rviz2",
				node_name="perception_rviz",
				arguments=['-d', str(self.pkg_dir / rviz_config_path)],
				namespace="/perception",
			)

	def _setup_scene(self):
		scene_type = self.config.scene_config.scene_type
		if scene_type == "generated":
			scene_generator = SceneGenerator(self.config.scene_config, self.logger.get_child('scene_generator'), self.pkg_dir)
			scene_generator.generate_scene()
		elif scene_type == "static":
			pass
		else:
			raise ValueError(f"Scene type {scene_type} not supported")
		
	def _launch_nodes(self):
		"""
		Launch all configured nodes as non-blocking subprocesses.
		"""
		for node_name, node_spec in self.active_nodes.items():
			cmd = self._node_to_command(node_spec)
			self.logger.debug(f"Launching node '{node_name}' with command: {' '.join(cmd)}")
			
			# For rviz2, avoid redirecting stdout/stderr so the GUI can pop up
			if node_spec["name"] == "perception_rviz":
				process = subprocess.Popen(cmd)
			else:
				process = subprocess.Popen(
					cmd,
					stdout=subprocess.PIPE,
					stderr=subprocess.PIPE,
					text=True,
					bufsize=1,
					universal_newlines=True
				)
				
				# # Start background threads to continuously log output
				# def log_output(pipe, prefix):
				# 	for line in pipe:
				# 			self._log_subprocess_output(line.strip(), f"{node_name} {prefix}")
			
				# Thread(target=log_output, args=(process.stdout, "STDOUT"), daemon=True).start()
				# Thread(target=log_output, args=(process.stderr, "STDERR"), daemon=True).start()
				
		self.active_processes[node_name] = process

	def _shutdown_nodes(self):
		"""
		Shutdown all subprocess nodes.
		"""
		for node_name, process in self.active_processes.items():
			self.logger.debug(f"Terminating node '{node_name}'")
			process.terminate()
		# Optionally, wait a short period for each process to exit gracefully.
		for node_name, process in self.active_processes.items():
			try:
				process.wait(timeout=5)
				self.logger.debug(f"Node '{node_name}' terminated gracefully.")
			except subprocess.TimeoutExpired:
				self.logger.debug(f"Node '{node_name}' did not terminate in time; killing it.")
				process.kill()


	def setup(self):
		self._setup_rviz_node()
		self._setup_scene()
		self._setup_scene_nodes()

	def execute(self):
		self._launch_nodes()
	
	def teardown(self):
		self._shutdown_nodes()


def main():
	global node
	executor = None
	try:
		# Initialize ROS2
		rclpy.init(args=None)
		
		# Create your node instance
		node = SceneViewer()
		
		# Create a single-threaded executor and add the node
		executor = rclpy.executors.SingleThreadedExecutor()
		executor.add_node(node)
		
		# Start the node's workload
		node.setup()
		node.execute()
		
		# Spin the executor (blocking call)
		executor.spin()
	except KeyboardInterrupt:
		print("Keyboard interrupt received, shutting down...")
		node.teardown()        
	except Exception as e:
		# Log the error with full traceback
		if node:
			node.get_logger().error(f"Unexpected error: {e}")
		print("Unexpected error occurred:")
		traceback.print_exc()
	finally:
		# Single cleanup point
		if node:
			try:
				node.teardown()
				# node.destroy_node()
			except Exception as e:
				print(f"Error during node cleanup: {e}")
				traceback.print_exc()
		if executor:
			executor.shutdown()

if __name__ == '__main__':
	main()