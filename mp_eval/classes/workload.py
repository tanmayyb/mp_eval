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
    goal_pos: List[float]
    start_orientation: List[float]
    goal_orientation: List[float]
@dataclass
class AgentConfig:
    detect_shell_radius: float
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
    experiment_type: str
    loop_frequency: int
    service_timeout: int
    delta_t: float
    max_prediction_steps: int
    poses: Poses # to be stored in start_goal.yaml
    agents: List[AgentConfig] # to be stored in scenario_config.yaml

@dataclass
class SceneParams:
    static_scene_path: str
    scene_generation_type: str
    scene_generation_params: Dict[str, float]

    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            static_scene_path=config['static_scene_path'] if 'static_scene_path' in config else None,
            scene_generation_type=config['scene_generation_type'] if 'scene_generation_type' in config else None,
            scene_generation_params=config['scene_generation_params'] if 'scene_generation_params' in config else None
        )
@dataclass
class SceneConfig:
    scene_type: str
    scene_params: SceneParams
    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            scene_type=config['scene_type'],
            scene_params=SceneParams.from_config(config['scene_params'])
        )
@dataclass
class FieldsConfig:
    k_circular_force: float
    agent_radius: float
    mass_radius: float
    max_allowable_force: float
    detect_shell_radius: float
    publish_force_vector: bool
    show_processing_delay: bool
@dataclass
class PerceptConfig:
    namespace: str
    mode: str
    scene_config: SceneConfig
    fields_config: FieldsConfig

    @classmethod
    def from_config(cls, config: Dict, planner_config: PlannerConfig):
        return cls(
            namespace=planner_config.experiment_type,
            mode=config['mode'],
            scene_config=SceneConfig.from_config(config['scene_config']),
            fields_config=FieldsConfig(**config['fields_config'])
        )

@dataclass
class WorkloadConfig:
    metadata: Metadata
    planner_config: PlannerConfig
    percept_config: PerceptConfig

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
                experiment_type=planner_data['experiment_type'],
                loop_frequency=planner_data['loop_frequency'],
                service_timeout=planner_data['service_timeout'],
                delta_t=planner_data['delta_t'],
                max_prediction_steps=planner_data['max_prediction_steps'],
                poses=poses,
                agents=agents
            )
            
            percept_data = data['percept_config']
            percept_config = PerceptConfig.from_config(percept_data, planner_config)

            return cls(
                metadata=metadata,
                planner_config=planner_config, 
                percept_config=percept_config
            )

