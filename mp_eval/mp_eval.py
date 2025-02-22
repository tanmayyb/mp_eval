#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import yaml
import signal
import traceback

from mp_eval.workload_manager import WorkloadManager
# from mp_eval.metrics_collector import MetricsCollector

class MPEval(Node):
    def __init__(self, node_name='mp_eval'):
        super().__init__(node_name)
        self.workload_manager = None
        # self.metrics_collector = None
        
        # Move setup and run to separate method that will be called after node is spinning
        self.setup()  # Keep setup in init for parameter declaration
        self.initialized = False

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

    def start(self):
        """Initialize and start the workload after node is spinning"""
        if self.initialized:
            return
            
        self.get_logger().info("Starting MPEval node...")
        try:
            # self.metrics_collector = MetricsCollector(self)
            self.workload_manager = WorkloadManager(
                self.get_logger().get_child('workload_manager'), 
                # self.metrics_collector
            )
            self.workload_manager.add_workload(self.workload)
            self.workload_manager.run()
            self.initialized = True
        except Exception as e:
            self.get_logger().error(f"Error starting workload: {str(e)}")
            raise

    def cleanup(self):
        """Cleanup function to ensure graceful shutdown"""
        self.get_logger().info("Starting cleanup...")
        try:
            if self.workload_manager:
                self.workload_manager.teardown()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
        self.get_logger().info("Cleanup completed")

# def main(args=None):
#     rclpy.init(args=args)
#     node = None

#     def signal_handler(sig, frame):
#         if node:
#             node.get_logger().info(f"Received signal {sig}, initiating shutdown...")
#             node.cleanup()
#         rclpy.shutdown()

#     # Register signal handlers
#     signal.signal(signal.SIGINT, signal_handler)
    
#     # try:
#     node = MPEval()
#     # Create a single-threaded executor
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)
    
#     # Start the workload after node is added to executor
#     node.start()
    
#     # Spin the executor instead of the node directly
#     executor.spin()
#     # except Exception as e:
#     #     if node:
#     #         node.get_logger().error(f"Unexpected error: {str(e)}")
#     #         # Make sure the error is printed
#     #         print(f"Error: {str(e)}")
#     # finally:
#     if node:
#         node.cleanup()
#         node.destroy_node()
#     executor.shutdown()
#     try:
#         rclpy.try_shutdown()
#     except Exception as e:
#         print(f"Error during ROS shutdown: {str(e)}")

# if __name__ == '__main__':
#     main()


def signal_handler(sig, frame):
    global node
    if node:
        node.get_logger().info(f"Received signal {sig}, initiating shutdown...")
        try:
            node.cleanup()
        except Exception as e:
            node.get_logger().error(f"Error during cleanup in signal handler: {e}")
            traceback.print_exc()
    rclpy.shutdown()

# Register the SIGINT handler
signal.signal(signal.SIGINT, signal_handler)

def main():
    global node
    executor = None
    try:
        # Initialize ROS2
        rclpy.init(args=None)
        
        # Create your node instance
        node = MPEval()
        
        # Create a single-threaded executor and add the node
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        # Start the node's workload
        node.start()
        
        # Spin the executor (blocking call)
        executor.spin()
        
    except Exception as e:
        # Log the error with full traceback
        if node:
            node.get_logger().error(f"Unexpected error: {e}")
        print("Unexpected error occurred:")
        traceback.print_exc()
        
    finally:
        # Ensure that the node is properly cleaned up and shutdown is called.
        if node:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception as e:
                print(f"Error during node cleanup: {e}")
                traceback.print_exc()
        if executor:
            executor.shutdown()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")
            traceback.print_exc()

if __name__ == '__main__':
    main()
