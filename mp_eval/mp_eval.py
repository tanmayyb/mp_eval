#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import yaml
import signal

from mp_eval.workload_manager import WorkloadManager
# from mp_eval.metrics_collector import MetricsCollector

class MPEval(Node):
    def __init__(self, node_name='mp_eval'):
        super().__init__(node_name)
        self.workload_manager = None
        # self.metrics_collector = None
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
        # self.metrics_collector = MetricsCollector(self)
        self.workload_manager = WorkloadManager(
            self.get_logger().get_child('workload_manager'), 
            # self.metrics_collector
        )
        self.workload_manager.add_workload(self.workload)
        self.workload_manager.run()

    def cleanup(self):
        """Cleanup function to ensure graceful shutdown"""
        self.get_logger().info("Starting cleanup...")
        try:
            # if self.metrics_collector:
            #     self.metrics_collector.stop()
            #     self.get_logger().info("Metrics collector stopped")
            if self.workload_manager:
                self.workload_manager.teardown()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
        self.get_logger().info("Cleanup completed")

def main(args=None):
    # Initialize with modified shutdown timeout parameters
    rclpy.init(args=args)
    node = None

    def signal_handler(sig, frame):
        if node:
            node.get_logger().info(f"Received signal {sig}, initiating shutdown...")
            node.cleanup()
        rclpy.shutdown()

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    # signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        node = MPEval()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {str(e)}")

if __name__ == '__main__':
    main()
