from mp_eval.assets.procedural.wall_generator import WallGenerator
import yaml
from typing import List, Dict
import numpy as np


def create_dense_wall(wall_name:str, wall_config:dict) -> Dict:

    assert 'thickness' in wall_config, "thickness is required for dense wall"
    # assert wall_config['thickness'] is int, "thickness must be an integer"
    thickness = wall_config['thickness']
    spacing = wall_config['spacing']
    wall_offset_k = wall_config['wall_offset_k']
    assert thickness > 1, "thickness must be greater than 1"
    assert spacing > 0.0, "spacing must be greater than 0"


    start = wall_offset_k - (float(thickness) - 1.0) * spacing / 2.0
    generators = {}
    for i in range(thickness):
        config = wall_config.copy()
        config['wall_offset_k'] = start + float(i) * spacing
        generators[f'{wall_name}_wall{i}'] = WallGenerator(config)
    return generators
   
class WallsGenerator:
    def __init__(self, scene_config_params:dict):
        self.scene_config_params = scene_config_params

    def _load_config(self):
        self.generators = {}
        for wall_name, wall_config in self.scene_config_params.items():
            if 'dense_wall' in wall_name:
                self.generators.update(create_dense_wall(wall_name, wall_config))
            elif 'wall' in wall_name:
                self.generators[wall_name] = WallGenerator(wall_config)
            else:
                raise ValueError(f"Wall name {wall_name} not supported")

    def _generate_all(self, generate_raw)->List[Dict]:
        generated_assets = []
        for generator in self.generators.values():
            generated_assets.extend(generator.generate_procedurally(generate_raw=generate_raw))
        
        return generated_assets
    
    def generate_procedurally(self, generate_raw=False) -> List[Dict]:
        self._load_config()
        return self._generate_all(generate_raw)
