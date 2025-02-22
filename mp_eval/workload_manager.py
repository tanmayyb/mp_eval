from mp_eval.classes.workload import WorkloadConfig
from mp_eval.classes.planner_interface import PlannerInterface
from mp_eval.classes.percept_interface import PerceptInterface
# from mp_eval.metrics_collector import MetricsCollector
import logging

class Workload:
    def __init__(self, config_path: str, logger):
        self.logger = logger
        self.config_path = config_path
        self.disable_planner = False
        self.disable_percept = False
        self.planner_interface = None
        self.percept_interface = None

        self.config = WorkloadConfig.from_yaml(self.config_path)
        self.workload_name = self.config.metadata.name
        self.planner_interface = PlannerInterface(self.config, self.logger.get_child('planner_interface'))
        self.percept_interface = PerceptInterface(self.config, self.logger.get_child('percept_interface'))

    def setup(self):
        if not self.disable_planner:
            self.planner_interface.setup()
        if not self.disable_percept:
            self.percept_interface.setup()
        self.logger.info(f"Workload setup complete")

    def execute(self):    
        if not self.disable_planner:
            self.planner_interface.execute()
        if not self.disable_percept:
            self.percept_interface.execute()

    def teardown(self):
        if not self.disable_planner:
            self.planner_interface.teardown()
        if not self.disable_percept:
            self.percept_interface.teardown()

    def run(self, disable_planner=False, disable_percept=False, disable_execute=False):
        self.disable_planner = disable_planner
        self.disable_percept = disable_percept
        self.setup()
        try:
            if not disable_execute:
                self.execute()
        except KeyboardInterrupt:
            self.logger.info("Workload execution interrupted by user")
            self.teardown()
        except Exception as e:
            self.logger.error(f"Error executing workload: {e}")
            self.teardown()

class WorkloadManager:
    def __init__(self, logger):
        self.workloads = []
        self.logger = logger

    def add_workload(self, workload_path: str):
        self.logger.info(f"Adding workload from {workload_path}")
        self.workloads.append(
            Workload(
                workload_path, 
                self.logger.get_child('workload')
            )
        )

    def run(self, disable_planner=False, disable_percept=False, disable_execute=False):
        for workload in self.workloads:
            workload.run(disable_planner, disable_percept, disable_execute)

    def teardown(self):
        for workload in self.workloads:
            try:
                workload.teardown()
            except Exception as e:
                self.logger.error(f"Error tearing down workload: {e}")
