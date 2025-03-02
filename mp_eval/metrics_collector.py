import logging
import os
import queue
import threading
from datetime import datetime
from functools import partial
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String, Float64

from mp_eval.classes.workload import WorkloadConfig


class MetricsCollectorNode(Node):
    """Node that collects and logs metrics from various ROS topics."""

    def __init__(self):
        super().__init__('metrics_collector')
        
        # Initialize parameters
        self._init_parameters()
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)
        
        # Initialize state variables
        self.stop_logging = False
        self.last_processed_time = {}
        self.primitives_logged = False
        
        # Setup logging infrastructure and callbacks
        self._setup_logging()
        self._setup_callbacks()
        self.logger.info("Metrics collector node initialized")

    def _init_parameters(self):
        """Initialize and process node parameters."""
        self.declare_parameter('ws_dir', '.')
        self.declare_parameter('workload', '.')
        self.declare_parameter('results_dir', './eval/results')
        self.declare_parameter('sampling_period', 0.5)
        
        # Get parameters and set environment variables
        self.ws_dir = Path(self.get_parameter('ws_dir').value)
        self.results_dir = Path(self.get_parameter('results_dir').value)
        self.throttle_period = self.get_parameter('sampling_period').value
        
        os.environ['WS_DIR'] = str(self.ws_dir)
        os.environ['RESULTS_DIR'] = str(self.results_dir)
        
        if not self.results_dir.exists():
            self.results_dir.mkdir(parents=True)
            self.logger.info(f"Created results directory: {self.results_dir}")

        # Load workload configuration
        self.workload_config_path = self.get_parameter('workload').value
        self.workload_config = WorkloadConfig.from_yaml(self.workload_config_path)

    def _setup_logging(self):
        """Setup logging infrastructure including file handling and worker thread."""
        # Create log directory
        self.workload_name = self.workload_config.metadata.label
        self.plan_name = self.workload_config.metadata.plan_name
        self.log_dir = self.results_dir / self.plan_name
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Create log file with timestamp
        current_time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.log_file = self.log_dir / f'{current_time}_{self.workload_name}.result'
        self.logger.info(f'Logging to {self.log_file}')

        # Initialize log file with header
        self.log_handle = open(self.log_file, 'w')
        with open(self.workload_config_path, 'r') as config_file:
            workload_yaml = yaml.safe_load(config_file)
        yaml.dump({'type': 'workload_config', 'workload_config': workload_yaml}, 
                 self.log_handle)
        self.log_handle.write('---\n')
        self.log_handle.flush()

        # Setup logging queue and worker thread
        self.log_queue = queue.Queue()
        self.log_thread = threading.Thread(target=self._log_worker)
        self.log_thread.start()

    def _setup_callbacks(self):
        """Setup ROS topic subscriptions."""
        namespace = self.workload_config.planner_config.experiment_type
        self.subs = {
            'pose': self.create_subscription(
                PoseStamped,
                f'/{namespace}/pose',
                self._agent_pose_callback,
                qos_profile_sensor_data
            ),
            'target': self.create_subscription(
                PoseStamped,
                f'/{namespace}/target',
                self._target_pose_callback,
                qos_profile_sensor_data
            ),
            'primitives': self.create_subscription(
                PointCloud2,
                '/primitives',
                self._process_primitives_msg,
                qos_profile_sensor_data
            )
        }

        self.subs['best_agent_name'] = self.create_subscription(
            String,
            f'/{namespace}/best_agent_name',
            self._process_best_agent_name_msg,
            qos_profile_sensor_data
        )

        for i in range(1, len(self.workload_config.planner_config.agents) + 1):
            self.subs[f'agent_{i}_planning_time'] = self.create_subscription(
                Float64,
                f'/{namespace}/agent_{i}/planning_time',
                partial(self._process_agent_planning_time_msg, agent_id=i),
                qos_profile_sensor_data
            )  

        for i in range(1, len(self.workload_config.planner_config.agents) + 1):
            self.subs[f'agent_{i}_cost'] = self.create_subscription(
                Float64,
                f'/{namespace}/agent_{i}/cost',
                partial(self._process_agent_cost_msg, agent_id=i),
                qos_profile_sensor_data
            )  

    def _log_worker(self):
        """Process log messages from queue until signaled to stop."""
        while not self.stop_logging:
            try:
                record = self.log_queue.get(timeout=0.5)
                if record is None:
                    break
                yaml.dump(record, self.log_handle)
                self.log_handle.write('---\n')
                self.log_handle.flush()
            except queue.Empty:
                continue

        # Drain remaining messages
        self._drain_log_queue()

    def _drain_log_queue(self):
        """Drain any remaining messages in the log queue."""
        while not self.log_queue.empty():
            try:
                record = self.log_queue.get_nowait()
                if record is None:
                    break
                yaml.dump(record, self.log_handle)
                self.log_handle.write('---\n')
                self.log_handle.flush()
            except queue.Empty:
                break

    def _agent_pose_callback(self, msg: PoseStamped):
        """Handle agent pose messages."""
        self._process_pose_msg(msg, 'agent')

    def _target_pose_callback(self, msg: PoseStamped):
        """Handle target pose messages."""
        self._process_pose_msg(msg, 'target')

    def _process_pose_msg(self, msg: PoseStamped, topic_id: str):
        """Process pose messages with throttling."""
        # Apply throttling
        current_time = self.get_clock().now().nanoseconds * 1e-9
        last_time = self.last_processed_time.get(topic_id, 0.0)
        if current_time - last_time < self.throttle_period:
            return

        self.last_processed_time[topic_id] = current_time

        if msg is not None:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            record = {
                'type': 'agent_pose' if topic_id == 'agent' else 'target_pose',
                'timestamp': timestamp,
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }
            self.log_queue.put(record)

    def _process_best_agent_name_msg(self, msg: String):
        """Process best agent name message."""
        record = {
            'type': 'best_agent_name',
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            'best_agent_name': msg.data
        }
        self.log_queue.put(record)

    def _process_agent_planning_time_msg(self, msg: Float64, agent_id: int):
        """Process agent planning time message."""
        record = {
            'type': 'agent_planning_time',
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            'planning_time': msg.data,
            'agent_id': agent_id
        } 
        self.log_queue.put(record)

    def _process_agent_cost_msg(self, msg: Float64, agent_id: int):
        """Process agent cost message."""
        record = {
            'type': 'agent_cost',
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            'cost': msg.data,
            'agent_id': agent_id
        } 
        self.log_queue.put(record)

    def _process_primitives_msg(self, msg: PointCloud2):
        """Process primitives pointcloud message (once only)."""
        if self.primitives_logged:
            return

        points = [[float(data[0]), float(data[1]), float(data[2])]
                 for data in pc2.read_points(msg, field_names=['x', 'y', 'z'], 
                                          skip_nans=True)]
        
        record = {
            'type': 'pointcloud',
            'timestamp': float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
            'num_points': len(points),
            'points': points
        }
        self.log_queue.put(record)
        self.primitives_logged = True

        # Unsubscribe from pointcloud topic
        self.destroy_subscription(self.subs['primitives'])

    def cleanup(self):
        """Perform clean shutdown of the node."""
        # Destroy all subscriptions
        for sub in self.subs.values():
            self.destroy_subscription(sub)

        # Stop logging thread
        self.stop_logging = True
        self.log_queue.put(None)
        self.log_thread.join(timeout=2.0)
        
        if self.log_thread.is_alive():
            self.logger.warn("Log worker did not finish in time, draining remaining messages synchronously.")
            self._drain_log_queue()

        self.log_handle.close()
        self.logger.info(f"\033[32mLogged to {self.log_file}\033[0m")
        self.logger.debug("Metrics collector node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()
