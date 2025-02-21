import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from functools import partial
from mp_eval.classes.workload import WorkloadConfig
import os
from pathlib import Path
from datetime import datetime
import yaml
import logging


class MetricsCollectorNode(Node):

    def __init__(self):
        super().__init__('metrics_collector')

        self.declare_parameter('ws_dir', '.')
        self.declare_parameter('workload', '.')
        self.declare_parameter('results_dir', './plans/results')

        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)


        # Get parameter values
        self.ws_dir = self.get_parameter('ws_dir').value
        self.results_dir = self.get_parameter('results_dir').value
        # Set workspace directory in environment
        os.environ['WS_DIR'] = self.ws_dir
        os.environ['RESULTS_DIR'] = self.results_dir

        self.ws_dir = Path(self.ws_dir)
        self.results_dir = Path(self.results_dir)

        # Create results directory if it doesn't exist
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            self.logger.info(f"Created results directory: {self.results_dir}")

        self.workload_config_path = self.get_parameter('workload').value
        self.workload_config = WorkloadConfig.from_yaml(self.workload_config_path)

        self.setup(self.workload_config, self.workload_config_path)

    def _setup_logfile(self):
        self.workload_name = self.workload_config.metadata.name
        self.log_dir = self.results_dir / self.workload_name
        self.log_dir.mkdir(parents=True, exist_ok=True)

        current_time = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.log_file = self.log_dir / f'{current_time}_{self.workload_name}.result'

        self.logger.info(f'Logging to {self.log_file}')
        # Open log file and write workload config as YAML
        with open(self.log_file, 'w') as f:
            with open(self.workload_config_path, 'r') as config_file:
                workload_yaml = yaml.safe_load(config_file)
            yaml.dump({'workload_config': workload_yaml}, f)

    def _setup_data_structures(self):
        self.agent_pose_data = []
        self.target_pose_data = []
        self.pointcloud_data = []

    def _setup_callbacks(self):
        namespace = self.workload_config.planner_config.experiment_type
        self.subs = {}
        self.subs['pose'] = self.create_subscription(
            PoseStamped,
            f'/{namespace}/pose',
            partial(self._dynamic_callback, topic='pose'),
            10
        )
        
        self.subs['target'] = self.create_subscription(
            PoseStamped,
            f'/{namespace}/target',
            partial(self._dynamic_callback, topic='target'),
            10
        )
        
        self.subs['primitives'] = self.create_subscription(
            PointCloud2,
            f'/primitives',
            partial(self._dynamic_callback, topic='primitives'),
            10
        )

    def _dynamic_callback(self, msg, topic: str):
        if topic == 'pose':
            self._process_pose_msg(msg, 'agent')
        elif topic == 'target':
            self._process_pose_msg(msg, 'target')
        elif topic == 'primitives':
            self._process_primitives_msg(msg)

    def _process_pose_msg(self, msg: PoseStamped, topic_id: str):
        if not msg is None:
            # Get ROS time from message header
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            # Extract pose data
            pose_data = {
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
            if topic_id == 'agent':
                self.agent_pose_data.append(pose_data)
            elif topic_id == 'target':
                self.target_pose_data.append(pose_data)

    def _process_primitives_msg(self, msg: PointCloud2):
        points = []
        for data in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
            # Convert numpy values to regular Python floats
            points.append([float(data[0]), float(data[1]), float(data[2])])

        # replace last pointcloud data with new one
        self.pointcloud_data = {
            'timestamp': float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
            'num_points': len(points),
            'points': points
        }

    def setup(self, workload_config: WorkloadConfig, workload_config_path: str):
        self.workload_config = workload_config
        self.workload_config_path = workload_config_path

        self._setup_logfile()
        self._setup_data_structures()
        self._setup_callbacks()

    def cleanup(self):
        # Destroy all subscriptions
        for sub in self.subs.values():
            self.destroy_subscription(sub)
        
        # Dump collected data to log file
        with open(self.log_file, 'a') as f:
            yaml.dump({
                'agent_pose_data': self.agent_pose_data,
                'target_pose_data': self.target_pose_data,
                'pointcloud_data': self.pointcloud_data
            }, f)
        
        self.logger.info(f'Metrics data saved to {self.log_file}')
        
        # Clear data structures
        self._setup_data_structures()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()
    