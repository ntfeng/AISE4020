# Imports
import pygame
import math
import user 
from obstacle import Obst_Rect as Rect
import sensor_sim
import map_processor

# Window rendering
class Camera:

    def __init__(self, user, dim):

        self.user = user
        self.dim = dim

    def update(self):

        self.pos = (self.user.pos[0] - self.dim[0] // 2, self.user.pos[1] - self.dim[1] // 2)

class Simulation:

    def __init__(self, map_path):
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
        self.obj_list = map_processor.load_map(map_path)
        self.cam = Camera(self.user_obj, (self.WIDTH, self.HEIGHT))
    
    def compute_slowdown(self, lidar_pts, user_pos, lidar_range, factor=0.05, cone_angle=30):

        # Initialize slowdown sums for each direction
        slowdown_sum = {"left": 0.0, "right": 0.0, "up": 0.0, "down": 0.0}

        for pt in lidar_pts:

            diff_x = pt[0] - user_pos[0]
            diff_y = pt[1] - user_pos[1]
            dist = math.hypot(diff_x, diff_y) # Hypotenuse
            if dist == 0 or dist > lidar_range:
                continue

            # Weight is stronger when closer
            weight = (lidar_range - dist) / lidar_range
            angle = math.degrees(math.atan2(diff_y, diff_x)) % 360 # Return range from -pi to pi radians

            # Check if the point is within the cone for each direction.
            # Right: around 0/360 degrees
            if angle <= cone_angle or angle >= 360 - cone_angle:
                slowdown_sum["right"] += weight
            # Left: around 180 degrees
            if abs(angle - 180) <= cone_angle:
                slowdown_sum["left"] += weight
            # Up: obstacles above give an angle around 270 degrees
            if abs(angle - 270) <= cone_angle:
                slowdown_sum["up"] += weight
            # Down: obstacles below give an angle around 90 degrees
            if abs(angle - 90) <= cone_angle:
                slowdown_sum["down"] += weight

        # Calculate slowdown multiplier
        slowdown = {}
        for d in slowdown_sum:
            slowdown[d] = max(0.1, 1 - factor * slowdown_sum[d])

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

    Simulation('floorplan2.png').run()

    pygame.quit()