from dataclasses import dataclass
import yaml
from typing import List, Dict, Tuple, Optional

@dataclass
class Metadata:
    name: str
    datetime: str
    description: str
@dataclass
class Poses:
    start_pos: List[float]
    end_pos: List[float]
    start_orientation: List[float]
    end_orientation: List[float]
@dataclass
class AgentConfig:
    mass: float
    radius: float
    max_velocity: float
    approach_distance: float
    k_attractor_force: float
    k_damping: float
    k_repel_force: float
    k_circular_force: float
    forces: List[str]
@dataclass
class PlannerConfig:
    planner_type: str
    loop_frequency: int
    poses: Poses # to be stored in start_goal.yaml
    agents: List[AgentConfig] # to be stored in scenario_config.yaml
@dataclass
class WorkloadConfig:
    metadata: Metadata
    planner_config: PlannerConfig
    # scene_config: SceneConfig

    @classmethod
    def from_yaml(cls, yaml_path: str):
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            
            metadata = Metadata(**data['metadata'])
            
            planner_data = data['planner_config']
            poses = Poses(**planner_data['poses'])
            
            agents = []
            for agent_data in planner_data['agents']:
                agent = AgentConfig(**agent_data)
                agents.append(agent)
                
            planner_config = PlannerConfig(
                planner_type=planner_data['planner_type'],
                loop_frequency=planner_data['loop_frequency'],
                poses=poses,
                agents=agents
            )
            
            return cls(
                metadata=metadata,
                planner_config=planner_config
            )

