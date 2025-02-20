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
    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            detect_shell_radius=config['detect_shell_radius'] if 'detect_shell_radius' in config else 0.0,
            mass=config['mass'] if 'mass' in config else 0.0,
            radius=config['radius'] if 'radius' in config else 0.0,
            max_velocity=config['max_velocity'] if 'max_velocity' in config else 0.0,
            approach_distance=config['approach_distance'] if 'approach_distance' in config else 0.0,
            k_attractor_force=config['k_attractor_force'] if 'k_attractor_force' in config else 0.0,
            k_damping=config['k_damping'] if 'k_damping' in config else 0.0,
            k_repel_force=config['k_repel_force'] if 'k_repel_force' in config else 0.0,
            k_circular_force=config['k_circular_force'] if 'k_circular_force' in config else 0.0,
            forces=config['forces'] if 'forces' in config else ['attractor_force'],
        )
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
    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            k_circular_force=config['k_circular_force'],
            agent_radius=config['agent_radius'],
            mass_radius=config['mass_radius'],
            max_allowable_force=config['max_allowable_force'],
            detect_shell_radius=config['detect_shell_radius'] if 'detect_shell_radius' in config else 0.0,
            publish_force_vector=config['publish_force_vector'] if 'publish_force_vector' in config else False,
            show_processing_delay=config['show_processing_delay'] if 'show_processing_delay' in config else False
        )
@dataclass
class RvizConfig:
    show_rviz: bool
    rviz_config_path: str
    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            show_rviz=config['show_rviz'] if 'show_rviz' in config else False,
            rviz_config_path=config['rviz_config_path'] if 'rviz_config_path' in config else None
        )
@dataclass
class PerceptConfig:
    namespace: str
    mode: str
    scene_config: SceneConfig
    fields_config: FieldsConfig
    rviz_config: RvizConfig
    @classmethod
    def from_config(cls, config: Dict, planner_config: PlannerConfig):
        return cls(
            namespace=planner_config.experiment_type,
            mode=config['mode'],
            scene_config=SceneConfig.from_config(config['scene_config']),
            fields_config=FieldsConfig.from_config(config['fields_config']),
            rviz_config=RvizConfig.from_config(config['rviz_config'])
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
                agent = AgentConfig.from_config(agent_data)
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

