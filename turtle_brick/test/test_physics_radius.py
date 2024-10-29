from turtle_brick.physics import World

def test_physics_radius():
  phys = World([1.0, 1.0, 1.0], 1.0, 1.0, 1.0)
  assert phys.radius == 1.0