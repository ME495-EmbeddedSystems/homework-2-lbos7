from turtle_brick.physics import World

def test_physics_dt():
  phys = World([1.0, 1.0, 1.0], 1.0, 1.0, 1.0)
  assert phys.dt == 1.0