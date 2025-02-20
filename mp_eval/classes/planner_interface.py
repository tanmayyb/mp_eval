import os
import subprocess
import time
import logging
from pathlib import Path
from threading import Thread
import yaml
from dataclasses import dataclass
from mp_eval.classes.workload import WorkloadConfig, PlannerConfig
        

class PlannerInterface:
    def __init__(self, config: WorkloadConfig, logger):
        self.config = config
        self.logger = logger
        self.logger.set_level(logging.DEBUG)
        self.log_path = Path(os.environ.get('RESULTS_DIR', '.')) / "planner.log"

        # docker container config
        self.image_name = 'percept-ga_cf_planner'
        self.container_name = 'ga_cf_planner'
        self.package_name = 'experiments'
        self.launch_command = None
        self.process = None


    def _create_workload_config(self):
        """Create scenario_config.yaml and start_goal.yaml from workload config"""
        ws_dir = os.environ['WS_DIR']
    
        planner_yaml = PlannerYaml.from_config(self.config.planner_config)
        
        if planner_yaml.config.experiment_type == "oriented_pointmass": # TODO: add other experiment types
            setup_config = planner_yaml.get_planner_setup_config()
            scenario_config = planner_yaml.get_scenario_config()
            config_dir = Path(ws_dir) / "src" / "ga_cf_planner" / "src" / "experiments" / "src" / "oriented_pointmass"
            setup_config_path = config_dir / "oriented_pointmass_config.yaml"
            scenario_config_path = config_dir / "start_goal.yaml"
            launch_file = "oriented_pointmass_launch.py"
        else:
            raise ValueError(f"Experiment type {planner_yaml.config.experiment_type} not supported")

        # Write the config files with indent=2
        with open(setup_config_path, 'w') as f:
            yaml.dump(setup_config, f, default_flow_style=False, sort_keys=False, indent=2)
        with open(scenario_config_path, 'w') as f:
            yaml.dump(scenario_config, f, default_flow_style=False, sort_keys=False, indent=2)
        
        self.launch_command = f"ros2 launch {self.package_name} {launch_file}"
        

    def _log_subprocess_output(self, output: str, prefix: str = ""):
        """Log subprocess output to both logger and file"""
        if output:
            with open(self.log_path, 'a') as f:
                f.write(f"\n=== {prefix} ===\n")
                f.write(output)


    def _setup_docker_container(self):
        try:
            ws_dir = os.environ['WS_DIR']
            
            # Check if container is running
            result = subprocess.run(
                ['docker-compose', 'ps', '-q', self.container_name],
                cwd=ws_dir,
                capture_output=True,
                text=True
            )
            self._log_subprocess_output(result.stdout, "DOCKER PS STDOUT")
            self._log_subprocess_output(result.stderr, "DOCKER PS STDERR")
            
            if not result.stdout.strip():
                self.logger.debug("Starting planner container with docker-compose")
                result = subprocess.run(
                    ['docker-compose', 'up', '-d', self.container_name],
                    cwd=ws_dir,
                    capture_output=True,
                    text=True,
                    check=True
                )
                self._log_subprocess_output(result.stdout, "DOCKER COMPOSE UP STDOUT")
                self._log_subprocess_output(result.stderr, "DOCKER COMPOSE UP STDERR")
            else:
                self.logger.debug("Planner container already running")
            
            # Check if ROS2 workspace is built
            check_ws = subprocess.run(
                ['docker', 'exec', self.container_name, 'test', '-d', 'install'],
                cwd=ws_dir,
                capture_output=True,
                text=True
            )
            self._log_subprocess_output(check_ws.stdout, "WORKSPACE CHECK STDOUT")
            self._log_subprocess_output(check_ws.stderr, "WORKSPACE CHECK STDERR")
            
            if check_ws.returncode != 0:
                self.logger.error("ROS2 workspace not built. Please build the workspace first.")
                raise RuntimeError("ROS2 workspace not found")
            else:
                self.logger.debug("ROS2 workspace is built")
            
        except subprocess.CalledProcessError as e:
            self._log_subprocess_output(str(e), "ERROR")
            self.logger.error(f"Docker-compose error: {str(e)}")
            raise

    def _execute_container_command(self):
        try:
            self.logger.debug("Launching planner node")
            ws_dir = os.environ['WS_DIR']

            # Launch the ROS2 node in the background
            launch_cmd = f". install/setup.bash && exec {self.launch_command}"
            self.process = subprocess.Popen(
                ['docker', 'exec', self.container_name, 'bash', '-c', launch_cmd],
                cwd=ws_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )

            # Give the planner time to start up
            time.sleep(0.5)
            
            # Check if process started successfully
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                self._log_subprocess_output(stdout, "PLANNER STDOUT")
                self._log_subprocess_output(stderr, "PLANNER STDERR")
                self.logger.error(f"Error launching planner")
                return
                
            # Start background thread to continuously log output
            def log_output(pipe, prefix):
                for line in pipe:
                    self._log_subprocess_output(line.strip(), prefix)
            
            Thread(target=log_output, args=(self.process.stdout, "PLANNER STDOUT"), daemon=True).start()
            Thread(target=log_output, args=(self.process.stderr, "PLANNER STDERR"), daemon=True).start()
            
            self.logger.debug("Planner node launched")
            
        except Exception as e:
            self._log_subprocess_output(str(e), "ERROR")
            self.logger.error(f"Container execution error: {str(e)}")


    def _teardown_docker_container(self):
        try:
            self.logger.debug("Stopping planner process")
            
            if self.process and self.process.poll() is None:
                # First try sending SIGINT for graceful shutdown
                self.process.send_signal(2)  # SIGINT
                try:
                    self.process.wait(timeout=10)  # Increased timeout for graceful shutdown
                except subprocess.TimeoutExpired:
                    self.logger.warning("Graceful shutdown failed, attempting SIGTERM")
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.logger.warning("SIGTERM failed, forcing kill with SIGKILL")
                        self.process.kill()
                        self.process.wait()
                
            self.logger.debug("Planner process terminated")

            # Add small delay to allow ROS2 to clean up
            time.sleep(1)
            
            # Stop the docker container
            ws_dir = os.environ['WS_DIR']
            result = subprocess.run(
                ['docker-compose', 'down', self.container_name],
                cwd=ws_dir,
                capture_output=True,
                text=True
            )
            self._log_subprocess_output(result.stdout, "DOCKER DOWN STDOUT")
            self._log_subprocess_output(result.stderr, "DOCKER DOWN STDERR")
            
        except Exception as e:
            self.logger.error(f"Error during teardown: {str(e)}")


    def setup(self):
        self._create_workload_config()
        self._setup_docker_container()

    def execute(self):
        self._execute_container_command()

    def teardown(self):
        self._teardown_docker_container()




@dataclass
class PlannerYaml:
    config: PlannerConfig

    @classmethod
    def from_config(cls, config: PlannerConfig):
        return cls(config)

    def get_planner_setup_config(self):
        # modify boilerplate with workload config
        scenario_config = {
            "loop_frequency": self.config.loop_frequency,
            "publishers": ["trajectory", "target", "pose"],
            "subscribers": [],
            "callback_clients": [
                "obstacle_heuristic_force",
                "velocity_heuristic_force",
                "goal_heuristic_force",
                "goalobstacle_heuristic_force",
                "random_heuristic_force"
            ],
            "callback_servers": [],
            "publisher": {
                "trajectory": {"type": "gafro_motor_vector", "topic": "trajectory", "callback_queue": "trajectory"},
                "target": {"type": "gafro_motor", "topic": "target", "callback_queue": "target"},
                "pose": {"type": "gafro_motor", "topic": "pose", "callback_queue": "pose"}
            },
            "callback_client": {
                "obstacle_heuristic_force": {
                    "type": "obstacle_heuristic_force",
                    "callback_request": "get_obstacle_heuristic_force",
                    "callback_response": "obstacle_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "velocity_heuristic_force": {
                    "type": "velocity_heuristic_force",
                    "callback_request": "get_velocity_heuristic_force",
                    "callback_response": "velocity_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "goal_heuristic_force": {
                    "type": "goal_heuristic_force",
                    "callback_request": "get_goal_heuristic_force",
                    "callback_response": "goal_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "goalobstacle_heuristic_force": {
                    "type": "goalobstacle_heuristic_force",
                    "callback_request": "get_goalobstacle_heuristic_force",
                    "callback_response": "goalobstacle_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "random_heuristic_force": {
                    "type": "random_heuristic_force",
                    "callback_request": "get_random_heuristic_force",
                    "callback_response": "random_heuristic_force_response",
                    "timeout": self.config.service_timeout
                }
            },
            "cf_planner": {
                "n_agents": len(self.config.agents),
                "agent_type": "pointmass", # TODO: make this dynamic
                "delta_t": self.config.delta_t,
                "max_prediction_steps": self.config.max_prediction_steps,
                "prediction_freq_multiple": 1,
                "approach_distance": 0.25,
                "k_workspace": 1.0,
                "k_goal_distance": 1.0,
                "k_path_length": 1.0,
                "k_safe_distance": 1.0,
                "workspace_limits": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        }

        # Add agent configurations
        for i, agent in enumerate(self.config.agents, 1):
            scenario_config["cf_planner"][f"agent_{i}"] = {
                "detect_shell_radius": agent.detect_shell_radius,
                "mass": agent.mass,
                "radius": agent.radius,
                "max_velocity": agent.max_velocity,
                "approach_distance": agent.approach_distance,
                "k_attractor_force": agent.k_attractor_force,
                "k_damping": agent.k_damping,
                "k_repel_force": agent.k_repel_force,
                "k_circular_force": agent.k_circular_force,
                "forces": agent.forces
            }

        return scenario_config
    
    def get_scenario_config(self):
        scenario_config = {
            "start_pos": self.config.poses.start_pos,
            "goal_pos": self.config.poses.goal_pos,
            "start_orientation": self.config.poses.start_orientation,
            "goal_orientation": self.config.poses.goal_orientation
        }
        return scenario_config