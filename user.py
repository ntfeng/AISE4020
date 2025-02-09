import pygame

# User 
class User:

    def __init__(self, pos=(0,0), speed=10):

        self.pos = pos
        self.speed = speed
        self.movement = [False, False, False, False]

    def input_handler(self):
        
        keys = pygame.key.get_pressed()
        self.movement[0] = keys[pygame.K_LEFT]  
        self.movement[1] = keys[pygame.K_RIGHT]  
        self.movement[2] = keys[pygame.K_UP] 
        self.movement[3] = keys[pygame.K_DOWN]

    def update(self):

        if self.movement[0]:  # Move left
            self.pos = (self.pos[0] - self.speed, self.pos[1])
        if self.movement[1]:  # Move right
            self.pos = (self.pos[0] + self.speed, self.pos[1])
        if self.movement[2]:  # Move up
            self.pos = (self.pos[0], self.pos[1] - self.speed)
        if self.movement[3]:  # Move down
            self.pos = (self.pos[0], self.pos[1] + self.speed)