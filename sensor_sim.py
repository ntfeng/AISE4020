import math
from shapely.geometry import Point, Polygon, LineString

class LiDAR_Sensor:

    def __init__(self, user, range=12, fov=360, speed=4500):

        self.range = range  # range measured in pixels
        self.speed = speed  # rotations per second
        self.user = user
        self.fov = fov  # field of vision (360 for a LiDAR)
        self.lidar_pts = []

    def simulate(self, num_rays, objs):

        new_lidar_pts = []
        user_coord = self.user.pos

        for sect in range(num_rays):
            # Calculate the ray angles
            angle = math.radians(sect * (self.fov / num_rays) - (self.fov / 2)) # Divides the FOV into sections
            # Hcos(ang)=A; Hsin(ang)=O where H is the range of the LiDAR
            cos_angle = math.cos(angle) # Used for horizontal coordinates
            sin_angle = math.sin(angle) # Used for vertical coordinates
            
            # Construct Shapely line to create ray
            end_point = (user_coord[0] + self.range * cos_angle, user_coord[1] + self.range * sin_angle)
            ray_line = LineString([user_coord, end_point])

            closest_distance = self.range 
            closest_point = None

            # Iterate through obstacles to compute intersection with the ray
            for obj in objs:
                # Want to use any 'stored' Shapely polygons first before having to instantiate one (processing power)
                poly = obj.shapely_poly if hasattr(obj, 'shapely_poly') else Polygon(obj.poly)
                inter_poly = poly.intersection(ray_line) # Returns poly shape of intersection
                
                if inter_poly.is_empty:
                    continue

                # Intersection geometry handler
                if inter_poly.geom_type == 'Point': # Handle case of line to line intersection
                    pt = (inter_poly.x, inter_poly.y)
                    dist = math.hypot(pt[0] - user_coord[0], pt[1] - user_coord[1])

                    if dist < closest_distance:
                        closest_distance = dist
                        closest_point = pt
                elif inter_poly.geom_type in ['MultiPoint', 'GeometryCollection']: # Handle the case of multiple intersections or grazing an edge
                    for geom in inter_poly.geoms:

                        if geom.geom_type == 'Point':
                            pt = (geom.x, geom.y)
                            dist = math.hypot(pt[0] - user_coord[0], pt[1] - user_coord[1])

                            if dist < closest_distance:
                                closest_distance = dist
                                closest_point = pt
                elif inter_poly.geom_type == 'LineString': # Handle the case of overlapping lines
                    
                    pt = inter_poly.interpolate(inter_poly.project(Point(self.user.pos)))
                    dist = math.hypot(pt.x - user_coord[0], pt.y - user_coord[1])
                    
                    if dist < closest_distance:
                        closest_distance = dist
                        closest_point = (pt.x, pt.y)

            if closest_point is not None:
                new_lidar_pts.append(closest_point)

        self.lidar_pts = new_lidar_pts
        return self.lidar_pts
