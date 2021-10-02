
# Odometry

[ ] Replace velocity motion model with just: motion model. All this does
    is take an odometry input (which also includes variance) and propagates
    the pose based on this.
[ ] Create an odometry data structure, that is a pose + variance, which is
    passed to the motion model.
[ ] Add a general "odometry sensor" to the Robot. This acts to represent the
    odometry estimate returned by the mobile robot base. On a real robot, this
    would be calculated by integrating encoder positions, possibly fused
    with data from an IMU (for orientation change) and GPS (for displacement).
    In simulation, just add a random error to the odometry to avoid the
    complexity of actually modelling how we calculate the odometry.
[ ] Add a placeholder for a visual odometry stage, which takes the odometry
    from the mobile robot as initialisation then optimises by aligning
    features or raw data between scans. (In 2D, will be ICP, but use
    visual odometry as a generic term).
    Visual odometry also needs a variance, which is found based on the
    sensor model (and/or feature model).
    Visual odometry may be fused with the initial odometry, which allows
    it to discount visual odometry when matching was poor, but override the
    base odometry when matching was particularly good.

# Ekf with landmarks and known map

[ ] Render point descriptors by drawing an array of lines surrounding
    the point, where the angle of each line encodes a dimension.
    (will be normalised from 0->1, so map this to -pi->pi)
    (Use different colours or each line to provide some distinction between
     them).
[ ] Define a feature extraction model.
[ ] Pass this to the feature extractor (in Lidar), such that we can see
    noise between the actual landmarks and observed, both in position and
    descriptor.
[ ] Implement a descriptor matcher function, which simply does naive nearest
    neighbour search of descriptors. If the feature extraction model has
    sufficient noise, we should see some incorrect matches.
[ ] Render correspondances between true landmarks and observed landmarks.
[ ] Implement the update step of ekf, by linearising the feature model.
    Create a function/class (whichever suitable) for the step and update
    steps of the kf/ekf.
    
# Ekf with raw data and known map

[ ] Create pure virtual base class for StateEstimator.
[ ] Add scan model, use in lidar.
[ ] Create scan ekf localisation which ignores descriptors (since we haven't
    added descriptors to scan data) and aims to maximise the probability of
    observed raw data - which ends up being equivalent to minimising the
    sum of square errors by scan aligning.
[ ] Use the predict step to initialise ICP. Investigate to see if you can
    get a covariance estimate out of scan alignment, and do a proper kalman
    filter update step by fusing prior estimate and information provided by
    scan.
    
# Particle filter with raw data and known map

[ ] Add user input to move the robot to a different point, producing
    the kidnapped robot problem. See that this makes ekf slam fail.
[ ] Implement particle filter slam with the standard update step at first,
    which also fails with kidnapping, but will succeed after you include
    a probability of kidnap in the motion model.
    + Once you add this probability of kidnap, can't linearise, so
      wouldn't be able to use this with ekf slam.

# Particle filter with raw data and unknown map
