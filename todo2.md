
# Ekf with landmarks and known map

[ ] Have the state estimator update pose covariance using the linearised
    model provided by the motion model.
    (And decide which frame to define variance in)
[ ] Render point descriptors by drawing an array of lines surrounding
    the point, where each line encodes 2 dimensions.
    In the case of an odd number of dimensions, set as horizontal line (y=0).
    With this, can tell that descriptors match if all the lines drawn around
    it align.
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

...
