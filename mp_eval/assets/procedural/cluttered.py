from typing import List, Dict
import random

class ClutteredGenerator:
    def __init__(
            self, 
            scene_config_params:dict,
            params_check:list = [
                # "spacing",
            ]
        ):
        self.params_check = params_check
        self.scene_config_params = scene_config_params
        self.invalid_config = False

        self._validate_config()
        if self.invalid_config:
            raise ValueError("Invalid configuration for cluttered generation")
            
    def _validate_config(self):
        params_check = self.params_check
        for param in params_check:
            if param not in self.scene_config_params:
                self.invalid_config = True
                return False
        return True
    
    def _load_config(self):
        self.num_obstacles = self.scene_config_params.get('num_obstacles', 1000)
        self.seed = self.scene_config_params.get('seed', 42)
        self.bbox = self.scene_config_params.get('bbox', [0.0, 0.0, 0.0, 1.0, 1.0, 1.0])

        self.include_deadzones = self.scene_config_params.get('include_deadzones', False)
        self.deadzone_bbox1 = self.scene_config_params.get('deadzone_bbox1', [0.0, 0.0, 0.0, 0.5, 0.5, 0.5])
        self.deadzone_bbox2 = self.scene_config_params.get('deadzone_bbox2', [0.0, 0.0, 0.0, 0.5, 0.5, 0.5])

    def _generate_cluttered(self) -> List[Dict]:
        def _is_inside_deadzone(point, deadzone):
            """Check if a point is inside the given deadzone bbox."""
            x, y, z = point
            x_min, y_min, z_min, x_max, y_max, z_max = deadzone
            return (x_min <= x <= x_max) and (y_min <= y <= y_max) and (z_min <= z <= z_max)

        random.seed(self.seed)
        round_to = 3
        x_min, y_min, z_min, x_max, y_max, z_max = self.bbox
        obstacles = []

        while len(obstacles) < self.num_obstacles:
            # Randomly generate obstacle position
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_min, z_max)

            if self.include_deadzones:
                if _is_inside_deadzone((x, y, z), self.deadzone_bbox1) or \
                   _is_inside_deadzone((x, y, z), self.deadzone_bbox2):
                    continue  # Skip this point if inside any deadzone

            # Store obstacle
            obstacles.append({"position": [round(x, round_to), round(y, round_to), round(z, round_to)]})

        return obstacles


    def generate_procedurally(self) -> List[Dict]:
        self._load_config()
        return self._generate_cluttered()
