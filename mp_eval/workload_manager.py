from mp_eval.classes.workload import WorkloadConfig
from mp_eval.classes.planner_interface import PlannerInterface
from mp_eval.classes.percept_interface import PerceptInterface

class Workload:
    def __init__(self, config_path: str, logger):
        self.config = WorkloadConfig.from_yaml(config_path)
        self.logger = logger
        self.planner_interface = PlannerInterface(self.config, self.logger.get_child('planner_interface'))
        self.percept_interface = PerceptInterface(self.config, self.logger.get_child('percept_interface'))

    def setup(self):
        self.planner_interface.setup()
        self.percept_interface.setup()

    def execute(self):
        self.planner_interface.execute()
        self.percept_interface.execute()

    def teardown(self):
        self.planner_interface.teardown()
        self.percept_interface.teardown()

    def run(self):
        self.setup()
        self.execute()
        self.teardown()

class WorkloadManager:
    def __init__(self, logger):
        self.workloads = []
        self.logger = logger

    def add_workload(self, workload_path: str):
        self.logger.info(f"Adding workload from {workload_path}")
        self.workloads.append(Workload(workload_path, self.logger.get_child('workload')))

    def run(self):
        for workload in self.workloads:
            workload.run()

    def teardown(self):
        for workload in self.workloads:
            workload.teardown()