import pygame
import math

import user
from obstacle import Obst_Rect as Rect
import sensor_sim

class Pathfinder:

    ''' Compute the projection of lidar point vector onto the desired direction vector'''
    def compute_vector_projection(self, pt, user_pos, desired_dir):

        # Project vector v onto u
        vector_v = (pt[0] - user_pos[0], pt[1] - user_pos[1])
        vector_u = (desired_dir[0] - user_pos[0], desired_dir[1] - user_pos[1])
        mag_u_squared = vector_u[0] ** 2 + vector_u[1] ** 2

        # In the case no desired direction, return user position
        if mag_u_squared == 0:
            return user_pos
        
        k = (vector_v[0] * vector_u[0] + vector_v[1] * vector_u[1]) / mag_u_squared

        # Vector must originate from user to projected location
        projection = (user_pos[0] + k * vector_u[0], user_pos[1] + k * vector_v[1])
        
        return projection
    
    '''Using projections, determine distance of lidar point to desired direction line'''
    def compute_dist(self, pt, user_pos, desired_dir):

        projection = self.compute_vector_projection(pt, user_pos, desired_dir)
        dist = math.hypot(pt[0] - projection[0], pt[1] - projection[1])

        return dist
    
    '''
        avoid_thresh => the minimum distance from the straight line required to consider the object a hazard
    '''
    def compute_repulsion_control_pt(self, user_pos, desired_dir, lidar_pts, avoid_thresh=30, repulsion_factor=0.5):

        repulsion_control_pt = (0, 0)

        # Iterate through every lidar point to determine respective distances and projections
        for pt in lidar_pts:
            dist = self.compute_dist(pt, user_pos, desired_dir)

            if dist < avoid_thresh:
                projection = self.compute_vector_projection(pt, user_pos, desired_dir)
                dir = (pt[0] - projection[0], pt[1] - projection[1])
                
                # Normalize vector so that only direction impacts point
                mag = math.hypot(dir[0], dir[1])

                if mag != 0:
                    dir = (dir[0] / mag, dir[1] / mag)
                else:
                    dir = (0, 0)

                # Want to 'punish' obstacles closer to user so amplify repulsion accordingly
                repulsion_amp = (avoid_thresh - dist) * repulsion_factor
                # Move away when positive and allow forward when negative (too far away to repulse)
                amped_repulsion_vector = (-repulsion_amp * dir[0], -repulsion_amp * dir[1])

                # Update control point
                repulsion_control_pt = (repulsion_control_pt[0] + amped_repulsion_vector[0], repulsion_control_pt[1] + amped_repulsion_vector[1])
        
        return repulsion_control_pt

    '''
    Bezier curves are widely used in computer graphics to model smooth curves
    Uses control points to calculate curve: P0 is start pt, p1 is control pt, p2 is end pt
    Return a list of points along the curve
    '''
    def compute_quad_bezier_curve(self, P0, P1, P2, num_pts=20):

        pts = []
        
        for i in range(num_pts):
            t = i / (num_pts - 1)
            x = (1 - t)**2 * P0[0] + 2 * (1 - t) * t * P1[0] + t**2 * P2[0]
            y = (1 - t)**2 * P0[1] + 2 * (1 - t) * t * P1[1] + t**2 * P2[1]
            pts.append((x, y))
        return pts
    
    def compute_path(self, user_pos, lidar_pts, lidar_range, user_movement, obst_priority_weight=0.05, user_priority_weight=1.0):

        up_cnt = 0
        down_cnt = 0
        left_cnt = 0
        right_cnt = 0

        # Determine repulsion vector based on the four quadrants
        for pt in lidar_pts:
            if pt[0] < user_pos[0] and pt[1] < user_pos[1]:
                left_cnt += 1
                up_cnt += 1
            elif pt[0] > user_pos[0] and pt[1] < user_pos[1]:
                right_cnt += 1
                up_cnt += 1
            elif pt[0] < user_pos[0] and pt[1] > user_pos[1]:
                left_cnt += 1
                down_cnt += 1
            elif pt[0] > user_pos[0] and pt[1] > user_pos[1]:
                right_cnt += 1
                down_cnt += 1

        # Determines vector of obstacles
        repulsion_vector = (right_cnt - left_cnt, down_cnt - up_cnt)

        desired_dir = [0, 0]
        if user_movement[0]:
            desired_dir[0] -= 1
        if user_movement[1]:
            desired_dir[0] += 1
        if user_movement[2]:
            desired_dir[1] -= 1
        if user_movement[3]:
            desired_dir[1] += 1

        # Normalize vector of desired direction
        mag = math.hypot(desired_dir[0], desired_dir[1])
        if mag != 0:
            desired_dir = (desired_dir[0] / mag, desired_dir[1] / mag)
        else:
            desired_dir = (0, 0)
        
        # Combine repulsion and desired direction vector
        net_vector = (user_priority_weight * desired_dir[0] - obst_priority_weight * repulsion_vector[0],
                      user_priority_weight * desired_dir[1] - obst_priority_weight * repulsion_vector[1])
    

        # Ensure that the repulsion does not completely override the desired direction
        if desired_dir[0] != 0 and net_vector[0] * desired_dir[0] < 0:
            net_vector = (desired_dir[0], net_vector[1])
        if desired_dir[1] != 0 and net_vector[1] * desired_dir[1] < 0:
            net_vector = (net_vector[0], desired_dir[1])

        # Normalize the net vector 
        net_mag = math.hypot(net_vector[0], net_vector[1])
        if net_mag != 0:
            net_direction = (net_vector[0] / net_mag, net_vector[1] / net_mag)
        else:
            net_direction = desired_dir
        
        # End point goes from user to range of lidar
        endpoint = (user_pos[0] + net_direction[0] * lidar_range,
                user_pos[1] + net_direction[1] * lidar_range)
    
        return endpoint, repulsion_vector, desired_dir, net_vector, net_direction