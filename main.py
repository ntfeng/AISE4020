# Imports
import pygame
import math

import user 
from obstacle import Obst_Rect as Rect
import sensor_sim

# Window rendering
class Camera:

    def __init__(self, user, dim):

        self.user = user
        self.dim = dim

    def update(self):

        self.pos = (self.user.pos[0] - self.dim[0] // 2, self.user.pos[1] - self.dim[1] // 2)

class Simulation:

    def __init__(self):
        # Pygame setup
        pygame.init()
        pygame.display.set_caption('LiDAR Simulation')

        self.WIDTH, self.HEIGHT = 1280, 720

        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()

        self.running = True

        # Variables
        # Colors
        self.RED = (255,0,0)
        self.WHITE = (255,255,255)
        # User Settings
        self.USER_SPEED = 10
        self.USER_RADIUS = 20
        # LiDAR Specs
        self.LiDAR_RANGE = 200 # Measured in pixels
        self.LiDAR_FOV = 360

        # Instantiate objects
        self.user_obj = user.User((self.WIDTH // 2, self.HEIGHT // 2), self.USER_SPEED)
        self.lidar = sensor_sim.LiDAR_Sensor(self.user_obj, self.LiDAR_RANGE, self.LiDAR_FOV, 4500)
        self.obj_list = [Rect((0, 0),(100,100)),
                         Rect((-100, 150),(50,100)),
                         Rect((-50, -50),(10,100)),]
        self.cam = Camera(self.user_obj, (self.WIDTH, self.HEIGHT))
    
    def compute_slowdown(self, lidar_pts, user_pos, lidar_range, movement_vector=(0,0), 
                        factor_wide=0.05, factor_thin=0.1, cone_angle=30, thin_count_threshold=5):
            # For each direction, store minimum distance, sum of weights, and count of points
            slowdown_data = {
                "left": {"min_dist": lidar_range, "sum_weight": 0.0, "count": 0},
                "right": {"min_dist": lidar_range, "sum_weight": 0.0, "count": 0},
                "up": {"min_dist": lidar_range, "sum_weight": 0.0, "count": 0},
                "down": {"min_dist": lidar_range, "sum_weight": 0.0, "count": 0}
            }
            
            # Normalize movement vector if the robot is moving
            mv_mag = math.hypot(movement_vector[0], movement_vector[1])
            if mv_mag > 0:
                mv_norm = (movement_vector[0] / mv_mag, movement_vector[1] / mv_mag)
            else:
                mv_norm = (0, 0)
            
            # Map each direction to its central angle.
            directions_angles = {
                "right": 0,
                "left": 180,
                "up": 270,
                "down": 90
            }
            
            for pt in lidar_pts:
                diff_x = pt[0] - user_pos[0]
                diff_y = pt[1] - user_pos[1]
                dist = math.hypot(diff_x, diff_y)
                if dist == 0 or dist > lidar_range:
                    continue
                
                # Weight increases as distance decreases.
                weight = (lidar_range - dist) / lidar_range
                
                angle = math.degrees(math.atan2(diff_y, diff_x)) % 360
                
                # Only consider obstacles in front of the current movement.
                if mv_mag > 0:
                    obs_norm = (diff_x / dist, diff_y / dist)
                    if (mv_norm[0] * obs_norm[0] + mv_norm[1] * obs_norm[1]) < 0:
                        continue
                
                # For each direction, check if the point falls within its cone.
                for direction, center_angle in directions_angles.items():
                    if direction == "right":
                        if angle <= cone_angle or angle >= 360 - cone_angle:
                            data = slowdown_data[direction]
                            data["count"] += 1
                            data["sum_weight"] += weight
                            if dist < data["min_dist"]:
                                data["min_dist"] = dist
                    else:
                        if abs(angle - center_angle) <= cone_angle:
                            data = slowdown_data[direction]
                            data["count"] += 1
                            data["sum_weight"] += weight
                            if dist < data["min_dist"]:
                                data["min_dist"] = dist
            
            # Compute slowdown multipliers per direction based on whether obstacles appear thin or wide.
            slowdown = {}
            for direction, data in slowdown_data.items():
                if data["count"] == 0:
                    slowdown[direction] = 1.0
                else:
                    # For thin obstacles (few points), use the closest distance with a higher sensitivity.
                    if data["count"] < thin_count_threshold:
                        slowdown_val = 1 - factor_thin * (lidar_range - data["min_dist"]) / lidar_range
                    # For wider obstacles, use the aggregated weight.
                    else:
                        slowdown_val = 1 - factor_wide * data["sum_weight"]
                    slowdown[direction] = max(0.1, slowdown_val)
            
            return slowdown

    
    def run(self):
        
        while self.running:
            self.screen.fill((0, 0, 0)) # Clear Screen

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Simulate Lidar and adjust speed based on proximity to any object
            lidar_pts = self.lidar.simulate(180, self.obj_list)

            # Compute slowdown multipliers based on LiDAR detections
            slowdown_dict = self.compute_slowdown(lidar_pts, self.user_obj.pos, self.LiDAR_RANGE)
        
            # Get user input to adjust location
            self.user_obj.input_handler()
            self.user_obj.update(slowdown=slowdown_dict)

            # Camera to 'correct' positioning of render
            self.cam.update()

            # Render
            for obj in self.obj_list:

                obj.render(self.screen, self.cam.pos, True)

            for pt in lidar_pts:

                adjusted_pt = (pt[0] - self.cam.pos[0], pt[1] - self.cam.pos[1])
                pygame.draw.circle(self.screen, self.RED, adjusted_pt, 2) 

            pygame.draw.circle(self.screen, self.WHITE, (self.WIDTH // 2, self.HEIGHT // 2), self.USER_RADIUS)

            pygame.display.update()
            self.clock.tick(60)      

if __name__ == "__main__":

    Simulation().run()

    pygame.quit()
