
# Problem

- I want to learn about localisation methods, with/without a known map and
  with/without landmarks.
- After reading theory, to properly understand these methods and their
  advantages/disadvantages, I need to implement them myself.
- To allow me to focus on the localisation algorithms, I want a minimal
  environment.
- To keep it simple, it will be only 2D.

# Requirements

- Model and visualise a 2D environment for a mobile robot, where:
    - A robot moves with a motion model p(x_k|u_k, x_{k-1})
    - A robot takes range measurements with a sensor model p(y_k|x_k, m_k)
- Control the robot to move to a target position, by setting u_k,
  with full knowledge of the pose and map, simply to actuate the
  robot and provide test data for state estimation.
- Be able to visualise all information relating to algorithms, including:
    - Raw measurements
    - Extracted features
    - Pose esimates with covariance
    - Velocity estimates with covariance
    - Individual components of an update step, relating to specific measurements
- Be able to step through an algorithm and see how the estimate changes
  with each step, or have the algorithm continue in realtime for a period
  of time.
- Make it easy to add different localisation / state estimation algorithms.

# Things to NOT do

- Don't make it 3D
- Don't spend time optimising algorithms to be fast, such as using a quadtree
  for querying an occupancy grid. Keep it simple and focus on understanding
  the algorithms.
- Don't model kinematics. Assume we can drive the robot velocity as desired,
  given motion constraints (ie: either diff-drive or holonomic).
  However, do model uncertainty in this motion model, which is taken into
  account in p(x_k | u_k, x_{k-1})
- Only use range measurements, and no more complex sensors. That means, no
  colour information.
- Don't model gps, imu, magnetometer or encoders/odometry.
    - Focus on state estimation via exteroceptive measurements.
    - The information provided by the proprioceptive sensors is used
      for pose tracking.
    - Don't bother with pose tracking. Instead, we take this into account
      with our motion model. Input u_k is the target velocity, and there is
      some error between this and the actual velocity.
- Don't implement lots of different types of state estimator. Implement what
  you deem to be the "best". If you find this has some drawbacks and want to
  test an alternative, then implement this one too.
