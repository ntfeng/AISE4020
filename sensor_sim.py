import math

class LiDAR_Sensor:

    def __init__(self, user, range=12, fov=360, speed=4500):

        self.range = range # range measured in metres
        self.speed = speed # rotations per second
        self.user = user
        self.fov = fov # field of vision (360 for a LiDAR)
        self.lidar_pts = []
    
    def simulate(self, objs):

        new_lidar_pts = []
        num_rays = 360

        for sect in range(num_rays):
            
            angle = math.radians(sect * (self.fov / 180.0) * math.pi - self.fov / 2.0) # Divides the FOV into sections ranging from -fov/2 to fov/2
            closest_intersect = None # Keep track of the closest intersection
            closest_pt = None # Keep track of the actual point of intersection

            # Simulate rays traveling from the LiDAR (check closest distance before further)
            for dist in range(self.range):

                ray_sim_coord = (int(self.user.pos[0] + dist * math.cos(angle)), int(self.user.pos[1] + dist * math.sin(angle)))
                
                # Check for collisions with any object (need positions of objects to simulate LiDAR)
                for obj in objs:

                    if self.is_object_near(ray_sim_coord, obj):

                        # If an intersection is found, update the closest intersection
                        if closest_intersect is None or dist < closest_intersect:

                            closest_intersect = dist
                            closest_pt = ray_sim_coord  # Store the point where we hit the object
                        break  # Stop checking further once a collision is detected
            
            # Add lidar point only if an intersection was found
            if closest_pt is not None:

                new_lidar_pts.append(closest_pt)
        
        self.lidar_pts = new_lidar_pts
        return self.lidar_pts
    
    def is_object_near(self, coord, obj):

        # Check if the point (x, y) is within the bounds of the object
        return obj.pos[0] <= coord[0] <= obj.pos[0] + obj.dim[0] and obj.pos[1] <= coord[1] <= obj.pos[1] + obj.dim[1]