# Imports
import pygame
import math

import user 
from obstacle import Obst_Rect as Rect
import sensor_sim
from pathfinder import Pathfinder as pf

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
        self.obj_list = [Rect((0, 0), (100, 100)),
                         Rect((-100, 150), (50, 100)),
                         Rect((-150, 100), (50, 100)),
                         Rect((-150, 250), (400, 30))]

        self.user_obj = user.User((self.WIDTH // 2, self.HEIGHT // 2), self.USER_SPEED)
        self.lidar = sensor_sim.LiDAR_Sensor(self.user_obj, self.LiDAR_RANGE, self.LiDAR_FOV, 4500)
        self.cam = Camera(self.user_obj, (self.WIDTH, self.HEIGHT))
        self.pathfinder = pf()

        self.control_strength = 3.0
    
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
            # self.user_obj.update()

            # Compute the minimum distance from the user to any obstacle
            min_distance = self.LiDAR_RANGE
            for pt in lidar_pts:
                dist = math.hypot(pt[0] - self.user_obj.pos[0], pt[1] - self.user_obj.pos[1])
                if dist < min_distance:
                    min_distance = dist

            # Compute the desired path using the Pathfinder class
            endpoint, repulsion_vector, desired_dir, net_vector, net_direction = self.pathfinder.compute_path(user_pos=self.user_obj.pos, lidar_pts=lidar_pts, lidar_range=self.LiDAR_RANGE, user_movement=self.user_obj.movement)
            
            # Compute bending intensity based on proximity
            threshold = 50
            dynamic_bend_intensity = 0.3 + 0.7 * (max(0, (threshold - min_distance)) / threshold)
            
            # Compute the perpendicular vector to the net direction
            perp_vector = (-net_direction[1], net_direction[0])
            rep_mag = math.hypot(repulsion_vector[0], repulsion_vector[1])
            offset_distance = rep_mag * dynamic_bend_intensity
            
            # Compute the midpoint between the user's position and the endpoint
            midpoint = ((self.user_obj.pos[0] + endpoint[0]) / 2,
                        (self.user_obj.pos[1] + endpoint[1]) / 2)
            
            # Offset the midpoint along the perpendicular to get the control point
            control_point = (midpoint[0] + offset_distance * perp_vector[0], midpoint[1] + offset_distance * perp_vector[1])
            
            # Adjust the control point using curve repulsion
            repulsion_offset = self.pathfinder.compute_repulsion_control_pt(user_pos=self.user_obj.pos, desired_dir=endpoint, lidar_pts=lidar_pts, avoid_thresh=30, repulsion_factor=0.5)
            control_point = (control_point[0] + repulsion_offset[0], control_point[1] + repulsion_offset[1])
            
            # Generate the quadratic Bezier curve
            curve_pts = self.pathfinder.compute_quad_bezier_curve(self.user_obj.pos, control_point, endpoint, num_pts=20)
            
            # Gently nudge the user towards the computed path. Only move them when user is moving
            if any(self.user_obj.movement):
                scaling_factor = (self.LiDAR_RANGE - min_distance) / self.LiDAR_RANGE
                nudge_strength = self.control_strength * scaling_factor

                guiding_idx = min(5, len(curve_pts) - 1)
                guiding_point = curve_pts[guiding_idx]
                guiding_vector = (guiding_point[0] - self.user_obj.pos[0], guiding_point[1] - self.user_obj.pos[1])

                # Normalize vector
                guiding_mag = math.hypot(guiding_vector[0], guiding_vector[1])
                if guiding_mag != 0:
                    guiding_vector = (guiding_vector[0] / guiding_mag, guiding_vector[1] / guiding_mag)
                else:
                    guiding_vector = (0, 0)
                
                # Perform nudging
                self.user_obj.pos = (self.user_obj.pos[0] + nudge_strength * guiding_vector[0], self.user_obj.pos[1] + nudge_strength * guiding_vector[1])

            # Camera to 'correct' positioning of render
            self.cam.update()

            # Render
            for obj in self.obj_list:
                obj.render(self.screen, self.cam.pos, True)

            # Render lidar points
            for pt in lidar_pts:
                adjusted_pt = (pt[0] - self.cam.pos[0], pt[1] - self.cam.pos[1])
                pygame.draw.circle(self.screen, self.RED, adjusted_pt, 2) 

            # Draw player
            pygame.draw.circle(self.screen, self.WHITE, (self.WIDTH // 2, self.HEIGHT // 2), self.USER_RADIUS)

            # Render the computed Bezier curve
            adjusted_curve = [(pt[0] - self.cam.pos[0], pt[1] - self.cam.pos[1]) for pt in curve_pts]
            for i in range(len(adjusted_curve) - 1):
                pygame.draw.line(self.screen, (0, 255, 0), adjusted_curve[i], adjusted_curve[i+1], 2)

            pygame.display.update()
            self.clock.tick(60)      

if __name__ == "__main__":

    Simulation().run()

    pygame.quit()