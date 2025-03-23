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
        self.font = pygame.font.SysFont(None, 24)
        # Colors
        self.RED = (255,0,0)
        self.WHITE = (255,255,255)
        # User Settings
        self.USER_SPEED = 10
        self.USER_RADIUS = 20
        # LiDAR Specs
        self.LiDAR_RANGE = 200 # Measured in pixels
        self.LiDAR_FOV = 360
        # Pathing
        self.curve_pts = []

        # Instantiate objects
        self.obj_list = [Rect((0, -250), (200, 400)),
                         Rect((-100, -200), (50, 500)),
                         Rect((-150, 100), (50, 100)),
                         Rect((-150, 250), (400, 30))]

        self.user_obj = user.User((self.WIDTH // 2, self.HEIGHT // 2), self.USER_SPEED)
        self.lidar = sensor_sim.LiDAR_Sensor(self.user_obj, self.LiDAR_RANGE, self.LiDAR_FOV, 4500)
        self.cam = Camera(self.user_obj, (self.WIDTH, self.HEIGHT))
        self.pathfinder = pf()

        # Button Variables
        button_w = 180
        button_h = 40
        gap = 10
        start_x = self.WIDTH - (button_w * 3 + gap * 2) - 20
        start_y = self.HEIGHT - button_h - 20

        # List of buttons along with respective attributes
        self.buttons = [
            {"name": "No Assistance", "strength": 0, "shape": pygame.Rect(start_x, start_y, button_w, button_h), "color": (0, 128, 0)},
            {"name": "Some Assistance", "strength": 3, "shape": pygame.Rect(start_x + (button_w + gap), start_y, button_w, button_h), "color": (255, 165, 0)},
            {"name": "Stronger Assistance", "strength": 6, "shape": pygame.Rect(start_x + 2*(button_w + gap), start_y, button_w, button_h), "color": (255, 0, 0)}
        ]

        # Default to control setting ("Some Assistance")
        self.ctrl_index = 1
        self.control_strength = self.buttons[self.ctrl_index]["strength"]
    
    def butt_event_handler(self, event):

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            for i, btn in enumerate(self.buttons):
                if btn["shape"].collidepoint(event.pos):
                    self.ctrl_index = i
                    self.control_strength = btn["strength"]
                    break
    
    def render_butts(self):

        for i, btn in enumerate(self.buttons):
            pygame.draw.rect(self.screen, btn["color"], btn["shape"])

            # Draw outline for selected button
            if i == self.ctrl_index:
                pygame.draw.rect(self.screen, (255, 255, 255), btn["shape"], width=3)

            # Render label in black
            text_surf = self.font.render(btn["name"], True, (0, 0, 0))
            text_rect = text_surf.get_rect(center=btn["shape"].center)
            self.screen.blit(text_surf, text_rect)

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

    def compute_steering_nudge(self, nudge_str):

        if not self.curve_pts:
            return (0, 0)

        guiding_idx = min(5, len(self.curve_pts) - 1)
        guiding_pt = self.curve_pts[guiding_idx]
        guiding_vector = (guiding_pt[0] - self.user_obj.pos[0], guiding_pt[1] - self.user_obj.pos[1])

        # Normalize vector
        guiding_mag = math.hypot(guiding_vector[0], guiding_vector[1])
        if guiding_mag != 0:
            guiding_vector = (guiding_vector[0] / guiding_mag, guiding_vector[1] / guiding_mag)
        else:
            guiding_vector = (0, 0)
        
        # Get user direction vector
        user_movement = self.user_obj.movement
        dx = (1 if user_movement[1] else 0) - (1 if user_movement[0] else 0)
        dy = (1 if user_movement[3] else 0) - (1 if user_movement[2] else 0)
        user_dir = (0, 0)
        user_dir_mag = math.hypot(dx, dy)
        if user_dir_mag > 0:
            user_dir = (dx / user_dir_mag, dy / user_dir_mag)
        else:
            user_dir = (0, 0)
        
        # Decompose vector into parallel and perpendicular components
        # Assuming vector denoted as v and u is unit vector
        # v = parallel - perpendicular
        # Parallel (v dot u)u; Perpendicular u(p x u)
        dot_prod = guiding_vector[0] * user_dir[0] + guiding_vector[1] * user_dir[1]
        parallel_vector = (dot_prod * user_dir[0], dot_prod * user_dir[1])
        perp_vector = (guiding_vector[0] - parallel_vector[0], guiding_vector[1] - parallel_vector[1])

        return (perp_vector[0] * nudge_str, perp_vector[1] * nudge_str)
        


    def pathfinder_logic(self):

        # Clear old curve
        self.curve_pts.clear()
        lidar_pts = self.lidar.simulate(180, self.obj_list)

        # Speed Policy
        # Slowdown factors are based on chosen setting. Should scale if option 1 or 2 is chosen
        if self.control_strength == 0:
            slowdown_dict = {'left': 1, 'right': 1, 'up': 1, 'down': 1}
        else:
            slowdown_dict = self.compute_slowdown(lidar_pts, self.user_obj.pos, self.LiDAR_RANGE)
        
        # Update user movement with slowdown
        self.user_obj.input_handler()
        self.user_obj.update(slowdown=slowdown_dict)

        # Compute the minimum distance from the user to any obstacle
        min_distance = self.LiDAR_RANGE
        for pt in lidar_pts:
            dist = math.hypot(pt[0] - self.user_obj.pos[0], pt[1] - self.user_obj.pos[1])
            if dist < min_distance:
                min_distance = dist
        
        # Compute the desired path using the Pathfinder class
        endpt, repulsion_vector, desired_dir, net_vector, net_direction = self.pathfinder.compute_path(user_pos=self.user_obj.pos, lidar_pts=lidar_pts, lidar_range=self.LiDAR_RANGE, user_movement=self.user_obj.movement)

        # Compute bending intensity based on proximity
        threshold = 50
        dynamic_bend_intensity = 0.3 + 0.7 * (max(0, (threshold - min_distance)) / threshold)
            
        # Compute the perpendicular vector to the net direction
        perp_vector = (-net_direction[1], net_direction[0])
        rep_mag = math.hypot(repulsion_vector[0], repulsion_vector[1])
        offset_distance = rep_mag * dynamic_bend_intensity
            
        # Compute the midpoint between the user's position and the endpoint
        midpt = ((self.user_obj.pos[0] + endpt[0]) / 2, (self.user_obj.pos[1] + endpt[1]) / 2)
            
        # Offset the midpoint along the perpendicular to get the control point
        control_pt = (midpt[0] + offset_distance * perp_vector[0], midpt[1] + offset_distance * perp_vector[1])
            
        # Adjust the control point using curve repulsion
        repulsion_offset = self.pathfinder.compute_repulsion_control_pt(user_pos=self.user_obj.pos, desired_dir=endpt, lidar_pts=lidar_pts, avoid_thresh=30, repulsion_factor=0.5)
        control_pt = (control_pt[0] + repulsion_offset[0], control_pt[1] + repulsion_offset[1])

        # Generate the quadratic Bezier curve
        self.curve_pts = self.pathfinder.compute_quad_bezier_curve(self.user_obj.pos, control_pt, endpt, num_pts=20)

        # Gently nudge the user towards the computed path. Only move them when user is moving
        if any(self.user_obj.movement):
            scaling_factor = (self.LiDAR_RANGE - min_distance) / self.LiDAR_RANGE
            nudge_strength = self.control_strength * scaling_factor
            steering_nudge = self.compute_steering_nudge(nudge_strength)
            
                
            # Perform nudging
            self.user_obj.pos = (self.user_obj.pos[0] + steering_nudge[0], self.user_obj.pos[1] + steering_nudge[1])
        
        return lidar_pts

    def run(self):
        
        while self.running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                else:
                    self.butt_event_handler(event)

            # Simulate Lidar and adjust speed based on proximity to any object
            lidar_pts = self.pathfinder_logic()

            # Camera to 'correct' positioning of render
            self.cam.update()

            # Render
            self.screen.fill((0, 0, 0)) # Clear Screen

            # Render obstacles
            for obj in self.obj_list:
                obj.render(self.screen, self.cam.pos, True)

            # Render lidar points
            for pt in lidar_pts:
                adjusted_pt = (pt[0] - self.cam.pos[0], pt[1] - self.cam.pos[1])
                pygame.draw.circle(self.screen, self.RED, adjusted_pt, 2) 

            # Draw player
            pygame.draw.circle(self.screen, self.WHITE, (self.WIDTH // 2, self.HEIGHT // 2), self.USER_RADIUS)

            # Render the computed Bezier curve
            if self.control_strength > 0 and self.curve_pts:
                adjusted_curve = [(pt[0] - self.cam.pos[0], pt[1] - self.cam.pos[1]) for pt in self.curve_pts]
                for i in range(len(adjusted_curve) - 1):
                    pygame.draw.line(self.screen, (0, 255, 0), adjusted_curve[i], adjusted_curve[i + 1], 2)

            # Render control buttons
            self.render_butts()

            pygame.display.update()
            self.clock.tick(60)      

if __name__ == "__main__":

    Simulation().run()

    pygame.quit()