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
                         Rect((-100, 150),(50,100))]
        self.cam = Camera(self.user_obj, (self.WIDTH, self.HEIGHT))
    
    def run(self):
        while self.running:
            self.screen.fill((0, 0, 0)) # Clear Screen

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Simulate Lidar and adjust speed based on proximity to any object
            lidar_pts = self.lidar.simulate(180, self.obj_list)
        
            # Get user input to adjust location
            self.user_obj.input_handler()
            self.user_obj.update()

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