from mp_eval.classes.workload import SceneConfig
from typing import List, Dict
import yaml
from pathlib import Path

class SceneGenerator:
    def __init__(self, config: SceneConfig, logger, pkg_dir: Path):
        self.config = config
        self.logger = logger
        self.generators = []
        self.assets = []
        self.pkg_dir = pkg_dir

    def _add_asset_generator(self, generator):
        self.generators.append(generator)

    def _generate_assets(self):
        scene_params = self.config.scene_params
        scene_generation_type = scene_params.scene_generation_type
        scene_generation_params = scene_params.scene_generation_params
       
        if scene_generation_type == "wall" \
            or scene_generation_type == "hole_in_the_wall":
            from mp_eval.assets.procedural.wall_generator import WallGenerator
            self._add_asset_generator(WallGenerator(scene_generation_params))
        elif scene_generation_type in ["walls", "narrow_passage", "narrowpassage", "trap"]:
            from mp_eval.assets.procedural.walls_generator import WallsGenerator
            self._add_asset_generator(WallsGenerator(scene_generation_params))
        elif scene_generation_type == "cluttered":
            from mp_eval.assets.procedural.cluttered import ClutteredGenerator
            self._add_asset_generator(ClutteredGenerator(scene_generation_params))
        elif scene_generation_type == "user":
            self.assets.append(scene_generation_params)
        else:
            raise ValueError(f"Scene generation type {scene_generation_type} not supported")

        for generator in self.generators:
            self.assets.append(generator.generate_procedurally())
    
    def get_assets(self):
        self._generate_assets()
        return self.assets

    def _asset_to_yaml(self, asset: List[Dict]):
        yaml_dict = {}
        yaml_dict['obstacles'] = asset
        return yaml.dump(yaml_dict)

    def _save_assets_as_yaml(self):
        asset = self.assets[0]
        asset_yaml = self._asset_to_yaml(asset)
        generated_assets_dir = self.pkg_dir / 'assets' / 'benchmark_scenes'
        generated_yaml_path = generated_assets_dir / 'auto_generated_scene.yaml'
        with open(generated_yaml_path, 'w') as f:
            f.write(asset_yaml)
        self.logger.debug(f"Saved assets to {generated_yaml_path}")

    def generate_scene(self):
        self._generate_assets()
        self._save_assets_as_yaml()
