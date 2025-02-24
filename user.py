import pygame
import math

# User 
class User:

    def __init__(self, pos=(0,0), speed=10):

        self.pos = pos
        self.speed = speed
        self.movement = [False, False, False, False]
        self.deadzone = 0.2

        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:

            self.joystick = None

    def input_handler(self):
        
        keys = pygame.key.get_pressed()
        self.movement[0] = keys[pygame.K_LEFT]  
        self.movement[1] = keys[pygame.K_RIGHT]  
        self.movement[2] = keys[pygame.K_UP] 
        self.movement[3] = keys[pygame.K_DOWN]

        if self.joystick:

            # Get joystick axis values for 360-degree motion 
            joystick_axis = [self.joystick.get_axis(0), self.joystick.get_axis(1)] # (Horizontal axis, Vertical axis)

            # Deadzone
            if abs(joystick_axis[0]) < self.deadzone:
                
                joystick_axis[0] = 0
            if abs(joystick_axis[1]) < self.deadzone:

                joystick_axis[1] = 0

            # Invert y-axis for proper movement (-ve to go up)
            joystick_axis[1] = -joystick_axis[1]   
            
            # Right movement (+ve x-axis)
            if joystick_axis[0] > 0:
                self.movement[1] = 1
                self.movement[0] = 0
            # Left movement (-ve x-axis)
            elif joystick_axis[0] < 0:
                self.movement[0] = 1
                self.movement[1] = 0
            else:
                self.movement[0] = 0
                self.movement[1] = 0

            # Up movement (+ve y-axis)
            if joystick_axis[1] > 0:
                self.movement[2] = 1
                self.movement[3] = 0
            # Down movement (-ve y-axis)
            elif joystick_axis[1] < 0:
                self.movement[3] = 1
                self.movement[2] = 0
            else:
                self.movement[2] = 0
                self.movement[3] = 0

    def update(self, slowdown=None):
        
        if slowdown is None:
            slowdown = {'left': 1, 'right': 1, 'up': 1, 'down': 1}
        if self.movement[0]:  # Move left
            self.pos = (self.pos[0] - self.speed * slowdown['left'], self.pos[1])
        if self.movement[1]:  # Move right
            self.pos = (self.pos[0] + self.speed * slowdown['right'], self.pos[1])
        if self.movement[2]:  # Move up
            self.pos = (self.pos[0], self.pos[1] - self.speed * slowdown['up'])
        if self.movement[3]:  # Move down
            self.pos = (self.pos[0], self.pos[1] + self.speed * slowdown['down'])