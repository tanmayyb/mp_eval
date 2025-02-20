from dataclasses import dataclass
import yaml
from typing import List, Dict, Tuple, Optional
from pathlib import Path
import subprocess
import os
import logging
import time
@dataclass
class Metadata:
    name: str
    datetime: str
    description: str
@dataclass
class Poses:
    start_pos: List[float]
    end_pos: List[float]
    start_orientation: List[float]
    end_orientation: List[float]
@dataclass
class AgentConfig:
    mass: float
    radius: float
    max_velocity: float
    approach_distance: float
    k_attractor_force: float
    k_damping: float
    k_repel_force: float
    k_circular_force: float
    forces: List[str]
@dataclass
class PlannerConfig:
    planner_type: str
    loop_frequency: int
    poses: Poses # to be stored in start_goal.yaml
    agents: List[AgentConfig] # to be stored in scenario_config.yaml
@dataclass
class WorkloadConfig:
    metadata: Metadata
    planner_config: PlannerConfig
    # scene_config: SceneConfig

    @classmethod
    def from_yaml(cls, yaml_path: str):
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            
            metadata = Metadata(**data['metadata'])
            
            planner_data = data['planner_config']
            poses = Poses(**planner_data['poses'])
            
            agents = []
            for agent_data in planner_data['agents']:
                agent = AgentConfig(**agent_data)
                agents.append(agent)
                
            planner_config = PlannerConfig(
                planner_type=planner_data['planner_type'],
                loop_frequency=planner_data['loop_frequency'],
                poses=poses,
                agents=agents
            )
            
            return cls(
                metadata=metadata,
                planner_config=planner_config
            )



class PlannerInterface:
    def __init__(self, config: WorkloadConfig, logger):
        self.config = config
        self.logger = logger
        self.logger.set_level(logging.DEBUG)
        self.image_name = 'percept-ga_cf_planner'
        self.container_name = 'ga_cf_planner'
        self.process = None
        self.launch_file = 'oriented_pointmass.py'

    def setup(self):
        self._create_config_files()
        # self._setup_docker_container()
        # create config file at location
        # subprocess:
        # check if docker container is running
        # or launch docker container
        # connect to docker container
        # check if build install directory exists
        # if not terminate workload. prompt user to build docker container


    def _create_config_files(self):
        pass

    # def execute(self):
    #     try:
    #         self.logger.debug("Launching planner node")
    #         ws_dir = os.environ['WS_DIR']

    #         # Launch the ROS2 node in the background
    #         launch_cmd = ". install/setup.bash && exec ros2 launch experiments oriented_pointmass.py"
    #         process = subprocess.run(
    #             ['docker', 'exec', '-d', self.container_name, 'bash', '-c', launch_cmd],
    #             cwd=ws_dir,
    #             capture_output=True,
    #             text=True,
    #             check=True
    #         )

    #         if process.stderr:
    #             self.logger.error(f"Error launching planner: {process.stderr}")
    #             return

    #         # Give the planner time to start up
    #         time.sleep(2)
    #         self.logger.debug("Planner node launched")
            
    #     except subprocess.CalledProcessError as e:
    #         self.logger.error(f"Failed to launch planner: {str(e)}")
    #     except Exception as e:
    #         self.logger.error(f"Container execution error: {str(e)}")


    # def _setup_docker_container(self):
    #     try:
    #         ws_dir = os.environ['WS_DIR']
            
    #         # Check if container is running
    #         result = subprocess.run(
    #             ['docker-compose', 'ps', '-q', self.container_name],
    #             cwd=ws_dir,
    #             capture_output=True,
    #             text=True
    #         )
            
    #         if not result.stdout.strip():
    #             self.logger.debug("Starting planner container with docker-compose")
    #             subprocess.run(
    #                 ['docker-compose', 'up', '-d', self.container_name],
    #                 cwd=ws_dir,
    #                 check=True
    #             )
    #         else:
    #             self.logger.debug("Planner container already running")
            
    #         # Check if ROS2 workspace is built
    #         check_ws = subprocess.run(
    #             ['docker', 'exec', self.container_name, 'test', '-d', 'install'],
    #             cwd=ws_dir,
    #             capture_output=True
    #         )
            
    #         if check_ws.returncode != 0:
    #             self.logger.error("ROS2 workspace not built. Please build the workspace first.")
    #             raise RuntimeError("ROS2 workspace not found")
    #         else:
    #             self.logger.debug("ROS2 workspace is built")
            
    #     except subprocess.CalledProcessError as e:
    #         self.logger.error(f"Docker-compose error: {str(e)}")
    #         raise


    # def teardown(self):
    #     try:
    #         self.logger.debug("Stopping ROS2 launch file")
    #         ws_dir = os.environ['WS_DIR']
            
    #         # Kill the ROS2 launch process
    #         subprocess.run(
    #             ['docker', 'exec', self.container_name, 'pkill', '-2', '-f', 'ros2 launch'],
    #             cwd=ws_dir,
    #             capture_output=True,
    #             check=True
    #         )
    #         time.sleep(1)  # Give it time to shutdown cleanly
            
    #         self.logger.debug("Planner process terminated")
    #     except subprocess.CalledProcessError as e:
    #         self.logger.error(f"Error during teardown: {str(e)}")

    def setup(self):
        self.test()

    def execute(self):
        pass

    def teardown(self):
        pass

    # def test(self):
    #     command = f"docker exec {self.container_name} /bin/bash -c 'source /opt/ros/rolling/setup.bash && ros2 launch experiments {self.launch_file}'"

    #     # Open log file for writing
    #     log_file = 'ros2_launch.log'

    #     with open(log_file, "w") as log:
    #         process = subprocess.Popen(command, shell=True, stdout=log, stderr=subprocess.STDOUT, text=True)

    #         print(f"Started ROS2 launch inside Docker. Logging output to {log_file}")

    #         try:
    #             process.wait()  # Wait for process to complete
    #         except KeyboardInterrupt:
    #             print("Terminating the ROS2 launch process...")
    #             process.terminate()






class Workload:
    def __init__(self, config_path: str, logger):
        self.config = WorkloadConfig.from_yaml(config_path)
        self.logger = logger
        self.planner_interface = PlannerInterface(self.config, self.logger.get_child('planner_interface'))

    def setup(self):
        self.planner_interface.setup()
        # setup perception

    def execute(self):
        # execute planner
        self.planner_interface.execute()
        # execute perception

    def teardown(self):
        # teardown planner
        # teardown perception
        # time.sleep(2)
        self.planner_interface.teardown()
        
    def run(self):
        self.setup()
        self.execute()
        self.teardown()
        # Construct the docker exec command



class WorkloadManager:
    def __init__(self, logger):
        self.workloads = []
        self.logger = logger

    def add_workload(self, workload_path: str):
        self.logger.info(f"Adding workload from {workload_path}")
        self.workloads.append(Workload(workload_path, self.logger.get_child('workload')))

    def run(self):
        for workload in self.workloads:
            workload.run()

