import pygame
import math
import numpy as np

import cv2
from shapely.geometry import Polygon
from shapely.ops import cascaded_union

class Sim_Map_Generator:

    def __init__(self, map, scale=1.0, merge_thresh=5, area_thresh=200, thickness=8, screen_width=1280, screen_height=720, close_kernel_size=(5, 5), close_iter=3, h_thresh=40, min_line_len=20, max_line_gap=15):
        
        self.map = map
        self.scale = scale

        self.merge_thresh = merge_thresh
        self.area_thresh = area_thresh
        self.thickness = thickness
        
        self.screen_width = screen_width
        self.screen_height = screen_height

        self.close_kernel_size = close_kernel_size 
        self.close_iter = close_iter 

        self.hough_thresh = h_thresh
        self.min_line_len = min_line_len
        self.max_line_gap = max_line_gap
    
    # Creates a skeleton for the walls to determine seperation points for polygon generation
    def gen_skeleton(self, preproc_map_cv2_img):

        # cv2 saves images as numpy
        skeleton = np.zeros(preproc_map_cv2_img.shape, np.uint8)

        '''Structure Element for Erosion and Dilation
        010
        111
        010'''
        struct_elem = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        temp = preproc_map_cv2_img.copy()

        # Erosion and dilation
        while cv2.countNonZero(temp) != 0:
            # Using concept of Opening
            eroded_img = cv2.erode(temp, struct_elem) # Erode to seperate shapes
            dilated_img = cv2.dilate(eroded_img, struct_elem) # Dilate to expand shapes but won't fully undo erosion
            cmp_img = cv2.subtract(temp, dilated_img) # Subtract iterative image with opened image; Captures the edges of each shape
            skeleton = cv2.bitwise_or(skeleton, cmp_img) # Saving the progress of the iterations (layering progress)
            temp = eroded_img.copy()
        
        return skeleton
    
    def merge_polys(self, poly_list):
    
        polys = []
        for poly in poly_list:

            if len(poly) >= 3: # Check number of vertices (min 3 for valid polygon)
                polys.append(Polygon(poly).buffer(self.merge_thresh)) # Buffer extends shapes so that close shapes overlap for merging

        if not polys:
            return []

        merged = cascaded_union(polys) # Performs union on the buffered shapes; Merges polygons that overlapping
        merged_polys = []

        if merged.geom_type == 'Polygon':
            merged_polys.append(list(merged.exterior.coords)) # Keep a list of exterior points
        elif hasattr(merged, 'geoms'): # Check if merge shape is MultiPolygon or GeometryCollection
            for geom in merged.geoms:
                
                try:
                    merged_polys.append(list(geom.exterior.coords))
                except AttributeError:
                    continue

        return merged_polys
    
    # Filter out polygons based on area threshold
    def filter_polys(self, poly_coords, area_thresh=None):

        if area_thresh is None:
            area_thresh = self.area_thresh

        filtered_polys = []
        for coords in poly_coords:
            poly = Polygon(coords)

            if poly.area >= area_thresh:
                filtered_polys.append(coords)

        return filtered_polys

    def thicken_poly(self, x1, y1, x2, y2, thickness=None):

        if thickness is None:
            thickness = self.thickness

        # Calculate length of "bone"
        x_diff = x2 - x1
        y_diff = y2 - y1
        length = math.hypot(x_diff, y_diff)

        if length == 0:
            return None

        # Compute a perpendicular unit vector to line to determine which direction to thicken polygon
        norm_x = -y_diff / length
        norm_y = x_diff / length
        half_thicc = thickness / 2
        offset_x = norm_x * half_thicc
        offset_y = norm_y * half_thicc

        # Compute the four corners of the thickened polygon
        pt1 = (x1 + offset_x, y1 + offset_y)
        pt2 = (x1 - offset_x, y1 - offset_y)
        pt3 = (x2 - offset_x, y2 - offset_y)
        pt4 = (x2 + offset_x, y2 + offset_y)

        return [pt1, pt2, pt3, pt4]
    
    def proc_img(self, img_path):

        img = cv2.imread(img_path)

        if img is None:
            print("Warning: Couldn't load image", img_path)
            return []

        img = cv2.resize(img, (self.screen_width, self.screen_height))

        gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_scale = cv2.GaussianBlur(gray_scale, (3, 3), 0) # Blur reduces noise

        # https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html#ga72b913f352e4a1b1b397736707afcde3
        # Adaptive threshold is good for different lightings which appears in the exported lidar data images
        proc_img = cv2.adaptiveThreshold(gray_scale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        # https://www.geeksforgeeks.org/python-opencv-morphological-operations/
        # Fill gaps
        kernel = np.ones(self.close_kernel_size, np.uint8)
        for _ in range(self.close_iter):
            proc_img = cv2.morphologyEx(proc_img, cv2.MORPH_CLOSE, kernel) # Closing (Dilation then erosion)
        
        skel_img = self.gen_skeleton(proc_img)

        # https://www.geeksforgeeks.org/python-opencv-canny-function/
        # Use Canny for edge detection
        edges = cv2.Canny(skel_img, 50, 150)

        # Probablistic Hough line detection
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=self.hough_thresh, minLineLength=self.min_line_len, maxLineGap=self.max_line_gap)
        
        line_segs = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                line_segs.append((x1, y1, x2, y2))

        return line_segs
    
    def scale_poly(self, poly, scale=None):

        if scale is None:
            scale = self.scale

        scaled_poly = [(x * scale - self.screen_width / 2 * (scale - 1), y * scale - self.screen_height / 2 * (scale - 1)) for (x, y) in poly]

        return scaled_poly
    
    def gen_map_polys(self):

        line_segs = self.proc_img(self.map)

        wall_polys = []
        for (x1, y1, x2, y2) in line_segs:
            poly = self.thicken_poly(x1, y1, x2, y2)

            if poly is not None:
                wall_polys.append(poly)

        merged_polys = self.merge_polys(wall_polys)
        filtered_polys = self.filter_polys(merged_polys)


        if self.scale != 1.0:
            scaled_polys = [self.scale_poly(poly) for poly in filtered_polys]
            return scaled_polys
        else:
            return filtered_polys
        # return filtered_polys

# For testing the class directly:
if __name__ == '__main__':

    map_gen = Sim_Map_Generator("maps/scan1_livingroom.png", scale=2.0)
    polygons = map_gen.gen_map_polys()
    print("Generated wall polygons:", len(polygons))

    pygame.init()
    pygame.display.set_caption("Map Polygons Display Test")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)
    screen = pygame.display.set_mode((map_gen.screen_width, map_gen.screen_height))

    running = True
    while running:
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

        screen.fill((0, 0, 0))
        for poly in polygons:
            pts = [(int(x), int(y)) for (x, y) in poly]
            pygame.draw.polygon(screen, (0, 0, 255), pts, width=2)

        text_surf = font.render(f"Polygons: {len(polygons)}", True, (255, 255, 255))
        screen.blit(text_surf, (20, 20))
        pygame.display.flip()

        clock.tick(60)

    pygame.quit()
        