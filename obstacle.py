import pygame

# Simulated Obstacle
class Obstacle:

    def __init__(self, pos, dim):

        self.pos = pos
        self.dim = dim
        self.BLACK = (0,0,0)
        self.BLUE = (0,0,255)

class Obst_Rect(Obstacle):

    def __init__(self, pos, dim):

        super().__init__(pos, dim)

    def render(self, screen, user_pos, is_vis=False):

        relative_pos = (self.pos[0] - user_pos[0], self.pos[1] - user_pos[1])

        if is_vis:
            render_col = self.BLUE
        else:
            render_col = self.BLACK

        pygame.draw.rect(screen, render_col, (relative_pos[0], relative_pos[1], self.dim[0], self.dim[1]))