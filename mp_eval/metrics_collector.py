from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2

from functools import partial
from mp_eval.classes.workload import WorkloadConfig
import os
from pathlib import Path
from datetime import datetime
import yaml
import logging


class MetricsCollector():

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger().get_child('metrics_collector')
        self.logger.set_level(logging.DEBUG)
        self.workload_config = None
        self.workload_config_path = None
        self.ws_dir = Path(os.environ['WS_DIR'])
        self.results_dir = Path(os.environ['RESULTS_DIR'])

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
        self.primitives_data = []

    def _setup_callbacks(self):
        namespace = self.workload_config.planner_config.experiment_type
        self.subs = {}
        self.subs['pose'] = self.node.create_subscription(
            PoseStamped,
            f'/{namespace}/pose',
            partial(self._dynamic_callback, topic='pose'),
            10
        )
        # self.subs['trajectory'] = self.node.create_subscription(
        #     Path,
        #     f'/{namespace}/trajectory',
        #     partial(self._dynamic_callback, topic='trajectory'),
        #     10
        # )
        
        self.subs['target'] = self.node.create_subscription(
            PoseStamped,
            f'/{namespace}/target',
            partial(self._dynamic_callback, topic='target'),
            10
        )
        
        self.subs['primitives'] = self.node.create_subscription(
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
            # self._primitives_msg = msg
            pass

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

    def setup(self, workload_config: WorkloadConfig, workload_config_path: str):
        self.workload_config = workload_config
        self.workload_config_path = workload_config_path

        self._setup_logfile()
        self._setup_data_structures()
        self._setup_callbacks()

    def stop(self):
        # Destroy all subscriptions
        for sub in self.subs.values():
            self.node.destroy_subscription(sub)
        
        # Dump collected data to log file
        with open(self.log_file, 'a') as f:
            yaml.dump({
                'agent_pose_data': self.agent_pose_data,
                'target_pose_data': self.target_pose_data,
                'primitives_data': self.primitives_data
            }, f)
        
        self.logger.info(f'Metrics data saved to {self.log_file}')
        
        # Clear data structures
        self._setup_data_structures()
    
    