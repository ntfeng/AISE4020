import pygame
from obstacle import Obst_Rect as Rect

def load_map(map_path):
    try:
        map_image = pygame.image.load(map_path).convert_alpha()
        print(f"Loaded map: {map_path}")
    except pygame.error as e:
        print(f"Error loading map: {e}")
        return []
    
    map_image = pygame.transform.scale(map_image, (1280, 720))

    width, height = map_image.get_size()
    map_surface = pygame.Surface((width, height), pygame.SRCALPHA)
    map_surface.blit(map_image, (0, 0))

    return extract_obstacles_get_at(map_surface, width, height)

def extract_obstacles_get_at(map_surface, width, height):
    obstacles = []
    visited = set()

    for x in range(width):
        for y in range(height):
            pixel_color = map_surface.get_at((x, y))
            # Check if it's black enough (threshold) or exactly black:
            if pixel_color == (0, 0, 0, 255) and (x, y) not in visited:
                rect_w, rect_h = find_rect_size_get_at(map_surface, width, height, x, y, visited)
                obstacles.append(Rect((x, y), (rect_w, rect_h)))

    return obstacles

def find_rect_size_get_at(map_surface, width, height, start_x, start_y, visited):
    rect_width = 0
    rect_height = 0

    # Expand width
    while (start_x + rect_width < width and 
           map_surface.get_at((start_x + rect_width, start_y)) == (0, 0, 0, 255)):
        rect_width += 1

    # Expand height
    while (start_y + rect_height < height and 
           map_surface.get_at((start_x, start_y + rect_height)) == (0, 0, 0, 255)):
        rect_height += 1

    # Mark pixels visited
    for i in range(rect_width):
        for j in range(rect_height):
            visited.add((start_x + i, start_y + j))

    return rect_width, rect_height
