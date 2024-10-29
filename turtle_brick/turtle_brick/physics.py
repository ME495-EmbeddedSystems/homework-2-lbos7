class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        ----
        brick : ([float64]) The (x,y,z) location of the brick
        gravity : (float64) - the acceleration due to gravity in m/s^2
        radius : (float64) - the radius of the platform
        dt : (float64) - timestep in seconds of the physics simulation

        """
        self.brick_location = brick
        self.gravity = gravity
        self.radius = radius
        self.dt = dt
        self.vel = 0

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
        ------
        (x,y,z) - location of the brick

        """
        return self.brick_location

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
        ----
        location : ([float64]) - the (x,y,z) location of the brick

        """
        self.brick_location = location
        self.vel = 0

    def drop(self):
        """Update the brick's location by having it fall in gravity for one timestep."""
        self.vel += self.gravity*self.dt
        self.brick_location[2] -= self.vel*self.dt + self.gravity*(self.dt ** 2)
