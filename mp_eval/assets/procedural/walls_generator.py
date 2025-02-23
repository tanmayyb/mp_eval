from mp_eval.assets.procedural.wall_generator import WallGenerator
import yaml
from typing import List, Dict

class WallsGenerator:
    def __init__(self, scene_config_params:dict):
        self.scene_config_params = scene_config_params

    def _load_config(self):
        self.generators = {}
        for wall_name, wall_config in self.scene_config_params.items():
            self.generators[wall_name] = WallGenerator(wall_config)

    def _generate_narrow_passage(self)->List[Dict]:
        generated_assets = []
        for generator in self.generators.values():
            generated_assets.extend(generator.generate_procedurally())
        
        return generated_assets
    
    def generate_procedurally(self) -> List[Dict]:
        self._load_config()
        return self._generate_narrow_passage()
