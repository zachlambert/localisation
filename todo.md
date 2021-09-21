
# Landmarks

[ ] Add landmarks to terrain definition
[ ] Add a function to get all visible features, where a feature has
    a range, angle and descriptor vector.

# Features

[ ] Add a FeatureExtractor class, from which you can choose a feature
    extraction method. For the moment, only implement a fake feature
    extraction method that gets all observable features from the terrain
    then adds noise. (and possibly adds some outliers and miss others).
[ ] Add a FeatureMatcher class, that takes an input of two vectors of features,
    (possibly with one being features of known landmarks), and identifies
    correspondances.

# Visualisation

[ ] Add covariance markers (ellipse for xy, segment for theta)
[ ] Add visualisation of landmarks on map (squares).
[ ] Add visualisation of features (raw measurements=circles, features=crosses).
[ ] Have the FeatureExtractor draw identified features (circle outline).
[ ] Have the FeatureMatcher draw lines between correspondances.

# State estimation

[ ] Have the state estimator track pose with a motion model, that
    adds a random error between target velocity and actual velocity.
    (Can also treat this as if the command velocity is like a measured
     incremental transform, such that we model the error between this
     transform (from pose tracking) and the actual incremental transform).
[ ] Add this to a state_estimator::EkfSlam class, which localises
    with a known map and known correspondances.
[ ] Implement the update step, which corrects the pose based on observed
    feature locations and expected locations (based on correspondances
    and map)
[ ] Modify EkfSlam to have unknown correspondances.
[ ] Implement state_estimator::ekf::EkfSlamMht, which uses multi-hypothesis
    tracking to make it robust to out
    correspondances may be incorrect, and there are outliers. With this,

# Models

Provide classes to provide function for probabilistic models.
Can set parameters for these, then pass to various parts of the program
where they are used (ie: simulation and in state estimation).
Keep the maths within these, have other classes (eg: Robot) only make
use of these functions.

Add these models in as required.

All the below models should provide function to:
- Evaluate the forward model, eg: p(x_k | x_{k-1}, u_k)
- Sample from the forward model
- Evaluate the inverse model, eg: p(u_k | x_{k-1}, x_k)
- Sample from the inverse model
- Linearise the forward model to get mean and covariance (if applicable)

[ ] Add a MotionModel class:
    - Forward: p(x_k | x_{k-1}, i_k) [+ Linearised]
    - Inverse: p(u_k | x_{k-1}, x_k)
[ ] Add a LidarModel class:
    - Forward: p({y_k} | x_k, m) [+ Linearised]
    - Inverse: p(x_k | {y_k}, m)
[ ] Add a FeatureModel class (when features are known)
    - Forward: p({f_k} | x_k, m)
    - Inverse: p(x_k | {f_k}, m)
