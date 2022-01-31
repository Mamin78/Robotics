def forward_kinematic(angular_velocity1, angular_velocity2, l, r):
  v = (r * angular_velocity1 / 2) + (r * angular_velocity2 / 2)
  t_d = (r * angular_velocity1 / 2*l) - (r * angular_velocity2 / 2*l)

  return v, t_d

def CCW_rotation(linear_velocity, robot_heading):
  return (math.cos(robot_heading) + math.sin(robot_heading)) * linear_velocity