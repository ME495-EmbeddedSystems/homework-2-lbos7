from turtle_brick.physics import World

def test_physics_location():
  phys = World([1.0, 1.0, 1.0], 1.0, 1.0, 1.0)
  assert phys.brick_location == [1.0, 1.0, 1.0]