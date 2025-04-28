import os
import subprocess
import time
import logging
from pathlib import Path
from threading import Thread
import yaml
from dataclasses import dataclass
from mp_eval.classes.workload import WorkloadConfig, PlannerYaml
from datetime import datetime

class PlannerInterface:
    def __init__(self, config: WorkloadConfig, logger):
        self.config = config
        self.logger = logger
        self.logger.set_level(logging.DEBUG)
        self.log_path = Path(os.environ.get('RESULTS_DIR', '.')) / "planner.log"
        self.workload_name = config.metadata.label

        # Clear the log file on startup
        with open(self.log_path, 'w') as f:
            f.write(f"=== Planner Log Started at {time.strftime('%Y-%m-%d %H:%M:%S')} for Workload: {self.workload_name} ===\n")

        # docker container config
        self.image_name = 'percept-ga_cf_planner'
        self.container_name = 'ga_cf_planner'
        self.package_name = 'experiments'
        self.launch_command = None
        self.process = None
        self.vtune_process = None
        self.vtune_result_dir = 'traces/vtune'

    def _create_workload_config(self):
        """Create scenario_config.yaml and start_goal.yaml from workload config"""
        ws_dir = os.environ['WS_DIR']
    
        planner_yaml = PlannerYaml.from_config(self.config.planner_config)
        experiment_type = planner_yaml.config.experiment_type
        if experiment_type == "oriented_pointmass": # TODO: add other experiment types
            setup_config = planner_yaml.get_planner_setup_config()
            scenario_config = planner_yaml.get_scenario_config()
            config_dir = Path(ws_dir) / "src" / "ga_cf_planner" / "src" / "experiments" / "src" / "oriented_pointmass"
            setup_config_path = config_dir / "oriented_pointmass_config.yaml"
            scenario_config_path = config_dir / "start_goal.yaml"
            launch_file = "oriented_pointmass_launch.py"
            run_profiler = self.config.metadata.run_planner_profiler
            self.target_node_name = "oriented_pointmass"
        else:
            raise ValueError(f"Experiment type {planner_yaml.config.experiment_type} not supported")

        # Write the config files with indent=2
        with open(setup_config_path, 'w') as f:
            yaml.dump(setup_config, f, default_flow_style=False, sort_keys=False, indent=2)
        with open(scenario_config_path, 'w') as f:
            yaml.dump(scenario_config, f, default_flow_style=False, sort_keys=False, indent=2)
        
        # self.launch_command = f"ros2 launch {self.package_name} {launch_file} enable_rviz:=false"

        self.launch_command = f"/ros2_ws/install/experiments/lib/experiments/{experiment_type} --ros-args -r __node:={self.target_node_name} -r __ns:=/{self.target_node_name}"


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
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_name = f"{self.workload_name}_{timestamp}"

            if self.config.metadata.run_planner_profiler:
                launch_cmd = f". install/setup.bash && exec vtune -collect threading -result-dir=/ros2_ws/{self.vtune_result_dir}/{result_name} -duration=20 -- {self.launch_command}"
                self.logger.debug(f"Launching planner node with vtune profiler")
            else:
                launch_cmd = f". install/setup.bash && exec {self.launch_command}"
                self.logger.debug(f"Launching planner node without vtune profiler")

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
                # Send SIGINT to all ROS processes in the container
                ws_dir = os.environ['WS_DIR']
                kill_cmd = f"docker exec {self.container_name} pkill -SIGINT -f ros2"
                subprocess.run(kill_cmd, shell=True, timeout=2)
                
                # Short wait for ROS nodes to shutdown
                time.sleep(0.5)
                
                # If process still running, force kill
                if self.process.poll() is None:
                    self.logger.warning("ROS shutdown incomplete, forcing kill")
                    self.process.kill()
                    self.process.wait(timeout=2)
                
            self.logger.debug("Planner process terminated")
            
            # Stop the docker container
            ws_dir = os.environ['WS_DIR']
            result = subprocess.Popen(
                ['docker-compose', 'down', self.container_name],
                cwd=ws_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Log output asynchronously
            Thread(target=lambda: self._log_subprocess_output(result.stdout.read(), "DOCKER DOWN STDOUT"), daemon=True).start()
            Thread(target=lambda: self._log_subprocess_output(result.stderr.read(), "DOCKER DOWN STDERR"), daemon=True).start()
            
        except Exception as e:
            self.logger.error(f"Error during teardown: {str(e)}")


    def setup(self):
        self._create_workload_config()
        self._setup_docker_container()

    def execute(self):
        self._execute_container_command()

    def teardown(self):
        self._teardown_docker_container()