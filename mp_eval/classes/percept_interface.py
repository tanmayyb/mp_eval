from typing import Dict, List, Optional, Any
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_service import LaunchService
from launch.actions import EmitEvent
from launch.events import Shutdown
from mp_eval.classes.workload import WorkloadConfig, PerceptConfig, SceneConfig
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from mp_eval.classes.scene_generator import SceneGenerator
import asyncio
import logging

class PerceptInterface:
    def __init__(self, config: WorkloadConfig, logger):
        """Initialize the ROS2 launch interface."""
        self.config = config.percept_config
        self.logger = logger
        self.logger.set_level(logging.DEBUG)
        self.active_nodes: Dict[str, Node] = {}
        self.launch_description = LaunchDescription()
        self.pkg_dir = Path(get_package_share_directory('percept'))
        self.launch_service: Optional[LaunchService] = None
        # self.launch_thread: Optional[threading.Thread] = None
        self.future: Optional[Any] = None
    
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
                # 'k_circular_force': self.config.fields_config.k_circular_force, # deprecated
                'k_cf_velocity': self.config.fields_config.k_cf_velocity,
                'k_cf_obstacle': self.config.fields_config.k_cf_obstacle,
                'k_cf_goal': self.config.fields_config.k_cf_goal,
                'k_cf_goalobstacle': self.config.fields_config.k_cf_goalobstacle,
                'k_cf_random': self.config.fields_config.k_cf_random,
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

    def _setup_scene(self):
        scene_type = self.config.scene_config.scene_type
        if not scene_type == "generated":
            return
        scene_generator = SceneGenerator(self.config.scene_config, self.logger.get_child('scene_generator'), self.pkg_dir)
        scene_generator.generate_scene()

    def _generate_launch_description(self):
        """Generate launch description based on configuration."""
        self._setup_scene_nodes()
        self._setup_fields_computer_node()
        self._setup_rviz_node()

    def _launch_nodes(self):
        """Launch all configured nodes."""
        for node in self.active_nodes.values():
            self.launch_description.add_action(node)

        # self._launch_nodes()
        # Create and configure the LaunchService instance.
        self.launch_service = LaunchService()
        self.launch_service.include_launch_description(self.launch_description)
        # Start the launch service asynchronously.
        self.logger.debug("Launching Percept LaunchService...")

        
        async def async_launch(launch_service):
            await launch_service.run_async()
        
        asyncio.run(async_launch(self.launch_service))
        
    def _shutdown_nodes(self):
        if self.launch_service is not None:
            self.logger.debug("Shutting down Percept LaunchService...")
            self.launch_service.shutdown()
            # If a future was returned, wait until it completes.
            if self.future is not None:
                self.future.result()
            self.logger.debug("Percept LaunchService shutdown complete.")

    def setup(self):
        self._setup_scene()
        self._generate_launch_description()

    def execute(self):
        self._launch_nodes()

    def teardown(self):
        self._shutdown_nodes()