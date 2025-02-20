from typing import Dict, List, Optional, Any
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_service import LaunchService
from launch.actions import EmitEvent
from launch.events import Shutdown
from mp_eval.classes.workload import WorkloadConfig, PerceptConfig
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


class PerceptInterface:
    def __init__(self, config: WorkloadConfig, logger):
        """Initialize the ROS2 launch interface."""
        self.config = config.percept_config
        self.logger = logger
        self.active_nodes: Dict[str, Node] = {}
        self.launch_description = LaunchDescription()
        self.pkg_dir = Path(get_package_share_directory('percept'))
    
    def _add_node(
        self,
        package_name: str,
        node_executable: str,
        node_name: str,
        parameters: Optional[Dict[str, Any]] = None,
        remappings: Optional[List[tuple]] = None,
        namespace: str = "",
        arguments: Optional[List[str]] = None
    ) -> Node:
        """
        Create a new node configuration.
        
        Args:
            package_name: Name of the ROS2 package
            node_executable: Name of the node executable
            node_name: Name to give the node instance
            parameters: Dictionary of parameters to pass to the node
            remappings: List of topic remappings as (from, to) tuples
            namespace: Namespace for the node
            arguments: Command line arguments to pass to the node
        """
        node = Node(
            package=package_name,
            executable=node_executable,
            name=node_name,
            arguments=arguments or [],
            parameters=[parameters] if parameters else None,
            remappings=remappings or [],
            namespace=namespace
        )
        self.active_nodes[node_name] = node
        return node

    def _launch_nodes(self):
        """Launch all configured nodes."""
        for node in self.active_nodes.values():
            self.launch_description.add_action(node)
        
        launch_service = LaunchService()
        launch_service.include_launch_description(self.launch_description)
        return launch_service.run()

    def _kill_node(self, node_name: str):
        """
        Kill a specific node by name.
        
        Args:
            node_name: Name of the node to kill
        """
        if node_name in self.active_nodes:
            shutdown_event = RegisterEventHandler(
                OnProcessExit(
                    target_action=self.active_nodes[node_name],
                    on_exit=[EmitEvent(event=Shutdown())]
                )
            )
            self.launch_description.add_action(shutdown_event)
            del self.active_nodes[node_name]

    def _kill_all_nodes(self):
        """Kill all active nodes."""
        node_names = list(self.active_nodes.keys())
        for node_name in node_names:
            self.kill_node(node_name)

    def _update_node_parameters(
        self,
        node_name: str,
        parameters: Dict[str, Any]
    ):
        """
        Update parameters for a specific node.
        
        Args:
            node_name: Name of the node to update
            parameters: New parameters dictionary
        """
        if node_name in self.active_nodes:
            node = self.active_nodes[node_name]
            updated_node = Node(
                package=node.package,
                executable=node.executable,
                name=node_name,
                parameters=[parameters],
                remappings=node.remappings,
                namespace=node.namespace
            )
            self.kill_node(node_name)
            self.active_nodes[node_name] = updated_node
            self.launch_description.add_action(updated_node)

    def _setup_scene_nodes(self):
        scene_type = self.config.scene_config.scene_type
        if scene_type == "static":
            scene_path = self.config.scene_config.scene_params.static_scene_path
            self.scene_node = self._add_node(
                package_name="percept",
                node_executable="scene_loader.py",
                node_name="scene_loader",
                parameters={
                    'obstacles_config_path': str(self.pkg_dir / scene_path)
                }
            )
        elif scene_type == "generated":
            scene_path = 'assets/benchmark_scenes/auto_generated_scene.yaml'
            self.scene_node = self._add_node(
                package_name="percept",
                node_executable="scene_loader.py",
                node_name="scene_loader",
                parameters={
                    'obstacles_config_path': str(self.pkg_dir / scene_path)
                }
            )
        else:
            raise ValueError(f"Scene type {self.config.scene_config.scene_type} not supported")
    
    def _setup_fields_computer_node(self):
        # Add fields computer node
        fields_computer = self._add_node(
            package_name="percept",
            node_executable="fields_computer",
            node_name="fields_computer",
            parameters={
                'k_circular_force': self.config.fields_config.k_circular_force,
                'agent_radius': self.config.fields_config.agent_radius,
                'mass_radius': self.config.fields_config.mass_radius,
                'max_allowable_force': self.config.fields_config.max_allowable_force,
                'detect_shell_rad': self.config.fields_config.detect_shell_radius,
                'publish_force_vector': self.config.fields_config.publish_force_vector,
                'show_processing_delay': self.config.fields_config.show_processing_delay
            },
            remappings=[
                ('/get_obstacle_heuristic_circforce', f'/{self.config.namespace}/get_obstacle_heuristic_force'),
                ('/get_goal_heuristic_circforce', f'/{self.config.namespace}/get_goal_heuristic_force'),
                ('/get_velocity_heuristic_circforce', f'/{self.config.namespace}/get_velocity_heuristic_force'),
                ('/get_goalobstacle_heuristic_circforce', f'/{self.config.namespace}/get_goalobstacle_heuristic_force'),
                ('/get_random_heuristic_circforce', f'/{self.config.namespace}/get_random_heuristic_force'),
            ]
        )

    def _setup_rviz_node(self):
        if self.config.rviz_config.show_rviz:
            # Add RViz node
            rviz_config_path = self.config.rviz_config.rviz_config_path
            rviz_node = self._add_node(
                package_name="rviz2",
                node_executable="rviz2",
                node_name="perception_rviz",
                parameters=None,
                arguments=['-d', str(self.pkg_dir / rviz_config_path)],
                namespace="perception",
            )

    def _generate_launch_description(self):
        """Generate launch description based on configuration."""
        self._setup_scene_nodes()
        self._setup_fields_computer_node()
        self._setup_rviz_node()

    def setup(self):
        self._generate_launch_description()

    def execute(self):
        self._launch_nodes()

    def teardown(self):
        self._kill_all_nodes()