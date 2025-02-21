#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import yaml

from mp_eval.workload_manager import WorkloadManager
from mp_eval.metrics_collector import MetricsCollector

class MPEval(Node):
    def __init__(self, node_name='mp_eval'):
        super().__init__(node_name)
        self.workload_manager = None
        # Declare parameters
        self.setup()
        self.run()

    def setup(self):
        self.declare_parameter('ws_dir', '.')
        self.declare_parameter('workload', '.')
        self.declare_parameter('results_dir', './plans/results')

        # Get parameter values
        self.ws_dir = self.get_parameter('ws_dir').value
        self.results_dir = self.get_parameter('results_dir').value
        self.workload = self.get_parameter('workload').value

        # Set workspace directory in environment
        os.environ['WS_DIR'] = self.ws_dir
        os.environ['RESULTS_DIR'] = self.results_dir

        # Create results directory if it doesn't exist
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            self.get_logger().info(f"Created results directory: {self.results_dir}")

    def run(self):
        self.metrics_collector = MetricsCollector(self)
        self.workload_manager = WorkloadManager(
            self.get_logger().get_child('workload_manager'), self.metrics_collector)
        self.workload_manager.add_workload(self.workload)
        self.workload_manager.run()

    def cleanup(self):
        if self.workload_manager:
            self.workload_manager.teardown()

def main(args=None):
    rclpy.init(args=args)
    node = MPEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
        node.cleanup()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
