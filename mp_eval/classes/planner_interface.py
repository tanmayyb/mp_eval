import os
import subprocess
import time
import logging
from pathlib import Path

from mp_eval.classes.workload import WorkloadConfig


class PlannerInterface:
    def __init__(self, config: WorkloadConfig, logger):
        self.config = config
        self.logger = logger
        self.logger.set_level(logging.DEBUG)
        self.image_name = 'percept-ga_cf_planner'
        self.container_name = 'ga_cf_planner'
        self.process = None
        self.launch_file = 'oriented_pointmass_launch.py'
        self.log_path = Path(os.environ.get('RESULTS_DIR', '.')) / "planner.log"

    def _log_subprocess_output(self, output: str, prefix: str = ""):
        """Log subprocess output to both logger and file"""
        if output:
            with open(self.log_path, 'a') as f:
                f.write(f"\n=== {prefix} ===\n")
                f.write(output)            
            # # Also log to debug logger
            # for line in output.splitlines():
            #     self.logger.debug(f"{prefix}: {line}")

    def setup(self):
        self._create_config_files()
        self._setup_docker_container()
        # create config file at location
        # subprocess:
        # check if docker container is running
        # or launch docker container
        # connect to docker container
        # check if build install directory exists
        # if not terminate workload. prompt user to build docker container


    def _create_config_files(self):
        pass

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

    def execute(self):
        try:
            self.logger.debug("Launching planner node")
            ws_dir = os.environ['WS_DIR']

            # Launch the ROS2 node in the background
            launch_cmd = ". install/setup.bash && exec ros2 launch experiments oriented_pointmass_launch.py"
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
            
            from threading import Thread
            Thread(target=log_output, args=(self.process.stdout, "PLANNER STDOUT"), daemon=True).start()
            Thread(target=log_output, args=(self.process.stderr, "PLANNER STDERR"), daemon=True).start()
            
            self.logger.debug("Planner node launched")
            
        except Exception as e:
            self._log_subprocess_output(str(e), "ERROR")
            self.logger.error(f"Container execution error: {str(e)}")


    def teardown(self):
        try:
            self.logger.debug("Stopping ROS2 launch file")
            
            if self.process and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)  # Wait up to 5 seconds for clean shutdown
                except subprocess.TimeoutExpired:
                    self.logger.warning("Planner process did not terminate gracefully, forcing kill")
                    self.process.kill()
                    self.process.wait()
                
            self.logger.debug("Planner process terminated")
            
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