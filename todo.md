
# UI

[ ] Write text for controls
[ ] Write text for all steps and highlight the current step
[ ] Build with ImGui
[ ] Add button to replace q/w/e inputs
[ ] Add inputs to dynamically reconfigure model/state_estimator parameters
Don't bother with adding input to dynamically change which controller to
use. 

# Localisation under known map

## Gmm (Gaussian mixture model) state estimator (also with landmarks/features)

Extend EKF State Estimator to maintain a number of gaussians.
Each undergo the same predict + update steps.
Need to calculate the probability of each component.
After this, perform a resampling step, where you randomly select new
gaussians from the old ones, with a pmf relating to their probabilities.
(ie: same as particle filter).

Note 1: With the resampling step, have a random probability of selecting a
random pose instead of resampling, to help with the kidnapped robot problem.

Note 2: When evaluating the state estimate, the simplest method is to average
out all the mixture means (weighted by their component probaility). However,
it might be worth using a clustering algorithm to identify components that
belong to the same mode. (eg: if we have two clusters of estimates, this is
bimodal, where there are two regions of possible location). After doing this, output the most likely pose as the state estimate. (Don't bother rendering averages of other clusters, rendering will be drawing all components anyway so you can see the clusters).

## Particle filter with landmarks

NOTE: Probably don't bother with this one. Using raw data with the particle
filter is more interesting. This is pretty much the same as the gmm filter
anyway, but worse.
The advantage of the particle filter, is that we don't need to linearise our
observations, so aren't limited to using features.

## Particle filter with raw data

Note: There may be room to adjust how the particles are sampled.
The ideal q(x) is f(x, u)g(y, x).
However, usually you use f(x, u) since it is difficult to sample from
g(x, y).
However, we can perform scan alignment to align our observations with the known
map (or the prior scan). This gives a mean pose from which we can sample around.

## Histogram filter with raw data

Maintain a histogram of p(x) about the mean pose. ie: Discretise p(x).
Only update this in the neighbourhood of our current estimate to avoid having to update over the whole map. (And make the grid aligned with the current state estimate).

If we want to handle an unknown initial position / kidnapping, can maintain a number of histogram estimates and 

## KLD-sampling

A modification of MCL (particle filter) that dynamically adjusts the number of particles used, depending on how many are needed. Makes it more efficient.


# Mapping

[ ] Add a generic map class. Think about what this will need to do. Also add rendering.
[ ] Implement a fixed-size occupancy grid. Use one of the previous localisation methods (with a map of known features) while updating the occupancy map - which provides more detail. (OR may just test without a state estimator, using the actual pose).

# SLAM

## EKF-SLAM

Feature based SLAM using EKF.
Online method (ie: every time it observes data, it processes this to update its estimate of the pose AND map).

## Graph-SLAM

Also feature-based, using gaussian methods.
However, is more flexible than EKF-SLAM.
Is an online method. It accumulates information (in the form of constraints), and then the optimisation is done offline, which optimises the pose and map estimate. (ie: optimises the pose graph).

## Sparse extended information filter

Optional.

## FastSLAM

Method using particle filters.
Optional.


## My idea of how to do SLAM

One of the previous methods may end up being equivalent to this method, but haven't looked into them much.
This method may also be too computationally expensive to be practical, but I think its worth having a look at.
There are probably various other reasons it won't work, but we will see.

Map: Update an occupancy map with latest observations and current (optimal) pose estimate, using the range model.
Predict: Propagage gaussian pose estimate with motion model.
Update: Localise within prior map, using the range model.
(repeat)

It is mathematically optimal and doesn't use features.
