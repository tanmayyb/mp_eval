import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from functools import partial
from mp_eval.classes.workload import WorkloadConfig
import os
from pathlib import Path
from datetime import datetime
import yaml
import logging
import queue
import threading
from rclpy.qos import qos_profile_sensor_data


class MetricsCollectorNode(Node):

    def __init__(self):
        super().__init__('metrics_collector')
        self.declare_parameter('ws_dir', '.')
        self.declare_parameter('workload', '.')
        self.declare_parameter('results_dir', './plans/results')

        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)

        # Get parameters and set environment variables
        self.ws_dir = self.get_parameter('ws_dir').value
        self.results_dir = self.get_parameter('results_dir').value
        os.environ['WS_DIR'] = self.ws_dir
        os.environ['RESULTS_DIR'] = self.results_dir
        self.ws_dir = Path(self.ws_dir)
        self.results_dir = Path(self.results_dir)
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            self.logger.info(f"Created results directory: {self.results_dir}")

        self.workload_config_path = self.get_parameter('workload').value
        self.workload_config = WorkloadConfig.from_yaml(self.workload_config_path)

        # Flag to signal log worker shutdown
        self.stop_logging = False

        # Dictionary to store last processed time for each topic
        self.last_processed_time = {}

        # Flag to indicate if a pointcloud has already been logged.
        self.primitives_logged = False

        # Setup asynchronous logging and subscriptions
        self._setup_logging()
        self._setup_callbacks()

    def _setup_logging(self):
        self.workload_name = self.workload_config.metadata.name
        self.log_dir = self.results_dir / self.workload_name
        self.log_dir.mkdir(parents=True, exist_ok=True)

        current_time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.log_file = self.log_dir / f'{current_time}_{self.workload_name}.result'
        self.logger.info(f'Logging to {self.log_file}')

        # Open the log file and write header info (workload config)
        self.log_handle = open(self.log_file, 'w')
        with open(self.workload_config_path, 'r') as config_file:
            workload_yaml = yaml.safe_load(config_file)
        header = {'type': 'workload_config', 'workload_config': workload_yaml}
        yaml.dump(header, self.log_handle)
        self.log_handle.write('---\n')
        self.log_handle.flush()

        # Create a thread-safe queue and start the logging thread.
        self.log_queue = queue.Queue()
        self.log_thread = threading.Thread(target=self._log_worker)
        self.log_thread.start()

        # Throttle period in seconds for pose/target messages
        self.throttle_period = 1.0

    def _log_worker(self):
        """
        Continuously processes log messages until signaled to stop.
        Uses a timeout so that it periodically checks if it should exit.
        """
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
        # Drain any final messages
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

    def _setup_callbacks(self):
        namespace = self.workload_config.planner_config.experiment_type
        self.subs = {}
        self.subs['pose'] = self.create_subscription(
            PoseStamped,
            f'/{namespace}/pose',
            self._agent_pose_callback,
            qos_profile_sensor_data
        )
        self.subs['target'] = self.create_subscription(
            PoseStamped,
            f'/{namespace}/target',
            self._target_pose_callback,
            qos_profile_sensor_data
        )
        self.subs['primitives'] = self.create_subscription(
            PointCloud2,
            '/primitives',
            self._process_primitives_msg,
            qos_profile_sensor_data
        )

    def _agent_pose_callback(self, msg: PoseStamped):
        self._process_pose_msg(msg, 'agent')

    def _target_pose_callback(self, msg: PoseStamped):
        self._process_pose_msg(msg, 'target')

    def _process_pose_msg(self, msg: PoseStamped, topic_id: str):
        # Throttle processing for this topic
        current_time = self.get_clock().now().nanoseconds * 1e-9
        last_time = self.last_processed_time.get(topic_id, 0.0)
        if current_time - last_time < self.throttle_period:
            return  # Skip processing if within throttle period
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

    def _process_primitives_msg(self, msg: PointCloud2):
        # Only log the pointcloud message once
        if self.primitives_logged:
            return

        points = []
        for data in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
            points.append([float(data[0]), float(data[1]), float(data[2])])
        record = {
            'type': 'pointcloud',
            'timestamp': float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
            'num_points': len(points),
            'points': points
        }
        self.log_queue.put(record)
        self.primitives_logged = True

        # Optionally, unsubscribe from the pointcloud topic as no further messages are needed
        self.destroy_subscription(self.subs['primitives'])

    def cleanup(self):
        """
        Clean shutdown: destroy subscriptions, signal the log worker to exit,
        wait briefly for it to finish, and then drain any leftover messages.
        """
        for sub in self.subs.values():
            self.destroy_subscription(sub)

        self.stop_logging = True
        self.log_queue.put(None)
        self.log_thread.join(timeout=2.0)
        if self.log_thread.is_alive():
            self.logger.warn("Log worker did not finish in time, draining remaining messages synchronously.")
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

        self.log_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
