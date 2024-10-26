class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self.brick = brick
        self.gravity = gravity
        self.radius = radius
        self.dt = dt

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """
        return self.brick

    @property.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self.brick = location

    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        self.brick[2] -= self.gravity*(self.dt ** 2)