from dataclasses import dataclass
import yaml
from typing import List, Dict, Tuple, Optional

@dataclass
class Metadata:
    created_at: str
    plan_name: str
    label: str
    tags: List[str]
    info: str
@dataclass
class Poses:
    start_pos: List[float]
    goal_pos: List[float]
    start_orientation: List[float]
    goal_orientation: List[float]
@dataclass
class AgentConfig:
    mass: float
    radius: float
    max_velocity: float
    approach_distance: float
    force_list: List[str]
    force_configs: Dict[str, Dict[str, float]]
    
    @classmethod
    def from_config(cls, config: Dict):
        force_configs = {}
        force_list = []

        for force_label, force_config in config['forces'].items():
            force_configs[force_label] = force_config
            force_list.append(force_label)

        return cls(
            mass=config['mass'] if 'mass' in config else 0.0,
            radius=config['radius'] if 'radius' in config else 0.0,
            max_velocity=config['max_velocity'] if 'max_velocity' in config else 0.0,
            approach_distance=config['approach_distance'] if 'approach_distance' in config else 0.0,
            force_list=force_list,
            force_configs=force_configs,
        )
@dataclass
class PlannerConfig:
    experiment_type: str
    loop_frequency: int
    service_timeout: int
    delta_t: float
    max_prediction_steps: int
    planning_frequency: int
    agent_switch_factor: float
    path_length_cost_weight: float
    goal_distance_cost_weight: float
    obstacle_distance_cost_weight: float
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
    # k_circular_force: float # deprecated
    k_cf_velocity: float
    k_cf_obstacle: float
    k_cf_goal: float
    k_cf_goalobstacle: float
    k_cf_random: float
    agent_radius: float
    mass_radius: float
    max_allowable_force: float
    detect_shell_radius: float
    publish_force_vector: bool
    show_processing_delay: bool
    show_requests: bool
    @classmethod
    def from_config(cls, config: Dict):
        return cls(
            # k_circular_force=config['k_circular_force'], # deprecated
            k_cf_velocity=config['k_cf_velocity'] if 'k_cf_velocity' in config else 0.0,
            k_cf_obstacle=config['k_cf_obstacle'] if 'k_cf_obstacle' in config else 0.0,
            k_cf_goal=config['k_cf_goal'] if 'k_cf_goal' in config else 0.0,
            k_cf_goalobstacle=config['k_cf_goalobstacle'] if 'k_cf_goalobstacle' in config else 0.0,
            k_cf_random=config['k_cf_random'] if 'k_cf_random' in config else 0.0,
            agent_radius=config['agent_radius'],
            mass_radius=config['mass_radius'],
            max_allowable_force=config['max_allowable_force'],
            detect_shell_radius=config['detect_shell_radius'] if 'detect_shell_radius' in config else 0.0,
            publish_force_vector=config['publish_force_vector'] if 'publish_force_vector' in config else False,
            show_processing_delay=config['show_processing_delay'] if 'show_processing_delay' in config else False,
            show_requests=config['show_requests'] if 'show_requests' in config else False
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
                planning_frequency=planner_data['planning_frequency'],
                agent_switch_factor=planner_data['agent_switch_factor'],
                poses=poses,
                agents=agents,
                path_length_cost_weight=planner_data['path_length_cost_weight'],
                goal_distance_cost_weight=planner_data['goal_distance_cost_weight'],
                obstacle_distance_cost_weight=planner_data['obstacle_distance_cost_weight']
            )
            
            percept_data = data['percept_config']
            percept_config = PerceptConfig.from_config(percept_data, planner_config)

            return cls(
                metadata=metadata,
                planner_config=planner_config, 
                percept_config=percept_config
            )


@dataclass
class PlannerYaml:
    config: PlannerConfig

    @classmethod
    def from_config(cls, config: PlannerConfig):
        return cls(config)

    def get_planner_setup_config(self):
        # modify boilerplate with workload config
        scenario_config = {
            "loop_frequency": self.config.loop_frequency,
            "publishers": [
                # "trajectory", 
                "target", "pose", 'best_agent_name']\
                    +[f"agent_{i}_planning_time" for i in range(1, len(self.config.agents) + 1)], # planning time for each agent
            "subscribers": [],
            "callback_clients": [
                "obstacle_heuristic_force",
                "velocity_heuristic_force",
                "goal_heuristic_force",
                "goalobstacle_heuristic_force",
                "random_heuristic_force",
                "obstacle_distance_cost_client"
            ],
            "callback_servers": [],
            "publisher": {
                # "trajectory": {"type": "gafro_motor_vector", "topic": "trajectory", "callback_queue": "trajectory"},
                "target": {"type": "gafro_motor", "topic": "target", "callback_queue": "target"},
                "pose": {"type": "gafro_motor", "topic": "pose", "callback_queue": "pose"},
                "best_agent_name": {"type": "ros_string", "topic": "best_agent_name", "callback_queue": "best_agent_name"},
                **{f"agent_{i}_planning_time": {"type": "ros_float64", "topic": f"agent_{i}/planning_time", "callback_queue": f"cf_planner/agent_{i}/planning_time"} for i in range(1, len(self.config.agents) + 1)}
            },
            "callback_client": {
                "obstacle_heuristic_force": {
                    "type": "obstacle_heuristic_force",
                    "callback_request": "get_obstacle_heuristic_force",
                    "callback_response": "obstacle_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "velocity_heuristic_force": {
                    "type": "velocity_heuristic_force",
                    "callback_request": "get_velocity_heuristic_force",
                    "callback_response": "velocity_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "goal_heuristic_force": {
                    "type": "goal_heuristic_force",
                    "callback_request": "get_goal_heuristic_force",
                    "callback_response": "goal_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "goalobstacle_heuristic_force": {
                    "type": "goalobstacle_heuristic_force",
                    "callback_request": "get_goalobstacle_heuristic_force",
                    "callback_response": "goalobstacle_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "random_heuristic_force": {
                    "type": "random_heuristic_force",
                    "callback_request": "get_random_heuristic_force",
                    "callback_response": "random_heuristic_force_response",
                    "timeout": self.config.service_timeout
                },
                "obstacle_distance_cost_client": {
                    "type": "obstacle_distance_cost",
                    "callback_request": "get_min_obstacle_distance",
                    "callback_response": "obstacle_distance_response",
                    "timeout": self.config.service_timeout
                }
            },
            "cf_planner": {
                "n_agents": len(self.config.agents),
                "agent_type": "pointmass", # TODO: make this dynamic
                "delta_t": self.config.delta_t,
                "max_prediction_steps": self.config.max_prediction_steps,
                "planning_frequency": self.config.planning_frequency,
                "prediction_freq_multiple": 1,
                "approach_distance": 0.25,
                "agent_switch_factor": self.config.agent_switch_factor,
                "costs": [
                    "path_length_cost",
                    "goal_distance_cost",
                    "obstacle_distance_cost"
                ],
                "path_length_cost": {
                    "weight": self.config.path_length_cost_weight
                },
                "goal_distance_cost": {
                    "weight": self.config.goal_distance_cost_weight
                },
                "obstacle_distance_cost": {
                    "weight": self.config.obstacle_distance_cost_weight
                }
            }
        }

        # Add agent configurations
        for i, agent in enumerate(self.config.agents, 1):
            scenario_config["cf_planner"][f"agent_{i}"] = {
                "mass": agent.mass,
                "radius": agent.radius,
                "max_velocity": agent.max_velocity,
                "approach_distance": agent.approach_distance,
                "forces": agent.force_list,
                **agent.force_configs
            }

        return scenario_config
    
    def get_scenario_config(self):
        scenario_config = {
            "start_pos": self.config.poses.start_pos,
            "goal_pos": self.config.poses.goal_pos,
            "start_orientation": self.config.poses.start_orientation,
            "goal_orientation": self.config.poses.goal_orientation
        }
        return scenario_config