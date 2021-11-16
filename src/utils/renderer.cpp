
#include "utils/renderer.h"
#include "utils/render_utils.h"


void drawPoseCovariance(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const Eigen::Matrix3d& cov,
    const double std_position_scaling=3,
    const double std_orientation_scaling=3)
{
    addCovarianceEllipse(
        vertex_array,
        pose,
        cov.block<2,2>(0,0),
        std_position_scaling,
        sf::Color::Green);
    addSegment(
        vertex_array,
        pose.position(),
        pose.orientation(),
        0.5,
        std_orientation_scaling * std::sqrt(cov(2,2)),
        sf::Color::Cyan);
}

void drawPose(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double size,
    sf::Color color)
{
    addEllipse(vertex_array, pose.position(), size, size, 0, color);
    addLine(
        vertex_array,
        pose.position(),
        pose.position() + 1.5*size*getDirection(pose.orientation()),
        LineType::ARROW,
        color,
        size*0.33);
}

void drawTerrain(
    sf::VertexArray& vertex_array,
    const Terrain& terrain)
{
    const sf::Color color(150, 150, 150);

    for (const auto& element: terrain.elements) {
        addMesh(
            vertex_array,
            element.pos,
            element.vertices,
            color
        );
    }
}

void drawPointCloud(
    sf::VertexArray& vertex_array,
    const PointCloud& point_cloud,
    MarkerType marker_type,
    double size,
    sf::Color color,
    const Pose& pose = Pose())
{
    for (const auto& point: point_cloud.points) {
        addMarker(
            vertex_array,
            transformPoint(pose, point.pos),
            marker_type,
            color,
            size);
    }

    // Also add descriptors. If not used, size = 0
    double descriptor_scaling = 0.8;
    for (size_t i = 0; i < point_cloud.points.size(); i++) {
        double inner_radius = size*0.5;
        double segment_width = 2*M_PI / point_cloud.points[i].descriptor.size();

        for (size_t j = 0; j < point_cloud.points[i].descriptor.size(); j++) {
            double angle = j * segment_width;
            double outer_radius = inner_radius + descriptor_scaling * point_cloud.points[i].descriptor(j);

            Eigen::Vector2d position = transformPoint(pose, point_cloud.points[i].pos);

            Eigen::Vector2d dir = getDirection(angle);
            addSegmentSlice(
                vertex_array,
                position,
                angle,
                inner_radius,
                outer_radius,
                segment_width * 0.5,
                color);
        }
    }
}

void drawPointCloudWithIndicators(
    sf::VertexArray& vertex_array,
    const PointCloud& point_cloud,
    const std::vector<bool>& indicators,
    MarkerType marker_type,
    double size,
    sf::Color color,
    const Pose& pose = Pose())
{
    bool use_indicators = (indicators.size() == point_cloud.points.size());

    for (size_t i = 0; i < point_cloud.points.size(); i++) {
        if (use_indicators && !indicators[i]) continue;
        addMarker(
            vertex_array,
            transformPoint(pose, point_cloud.points[i].pos),
            marker_type,
            color,
            size);
    }

    // Also add descriptors. If not used, size = 0
    double descriptor_scaling = 0.8;
    for (size_t i = 0; i < point_cloud.points.size(); i++) {
        if (use_indicators && !indicators[i]) continue;

        double inner_radius = size*0.5;
        double segment_width = 2*M_PI / point_cloud.points[i].descriptor.size();

        for (size_t j = 0; j < point_cloud.points[i].descriptor.size(); j++) {
            double angle = j * segment_width;
            double outer_radius = inner_radius + descriptor_scaling * point_cloud.points[i].descriptor(j);

            Eigen::Vector2d position = transformPoint(pose, point_cloud.points[i].pos);

            Eigen::Vector2d dir = getDirection(angle);
            addSegmentSlice(
                vertex_array,
                position,
                angle,
                inner_radius,
                outer_radius,
                segment_width * 0.5,
                color);
        }
    }
}

void drawMatches(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const FeatureMatcher::Result& match_result,
    sf::Color line_color)
{
    if (!match_result.known_features || !match_result.observed_features) return;

    for (size_t i = 0; i < match_result.matches.size(); i++) {
        addLine(
            vertex_array,
            match_result.known_features->points[match_result.matches[i].index_known].pos,
            transformPoint(pose, match_result.observed_features->points[match_result.matches[i].index_observed].pos),
            LineType::LINE,
            line_color,
            0.05);
    }
}

// Target
void drawTarget(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    double size,
    sf::Color color)
{
    addMarker(
        vertex_array,
        pose.position(),
        MarkerType::CROSS,
        color,
        size);
}

void drawStateEstimatorEkf(
    sf::VertexArray& vertex_array,
    const StateEstimatorEkf& state_estimator)
{
    drawPoseCovariance(
        vertex_array,
        state_estimator.estimate.pose,
        state_estimator.estimate.covariance);

    // Detected features
    drawPointCloud(
        vertex_array,
        state_estimator.features,
        MarkerType::RING,
        0.4,
        sf::Color::Green,
        state_estimator.getStateEstimate());

    if (state_estimator.match_result.known_features) {
        drawPointCloudWithIndicators(
            vertex_array,
            *state_estimator.match_result.known_features,
            state_estimator.match_result.indicators,
            MarkerType::RING,
            0.4,
            sf::Color::Blue,
            Pose());
    }

    // Correspondances
    drawMatches(
        vertex_array,
        state_estimator.getStateEstimate(),
        state_estimator.match_result,
        sf::Color::Black);
}

void drawStateEstimatorMht(
    sf::VertexArray& vertex_array,
    const StateEstimatorMht& state_estimator)
{
    for (auto& component: state_estimator.components) {
        double scale;
        if (component.log_likelihood < -100) {
            scale = 0.1;
        } else {
            scale = 0.1 + 0.9 * (component.log_likelihood + 100) / (-100);
        }

        drawPoseCovariance(
            vertex_array,
            component.pose,
            component.covariance,
            3 * scale,
            3 * scale);

        // Don't bother drawing features or correspondances - will be too clutterred.
        // Use standard EKF to observe how these behave
    }
}

void drawSim(sf::VertexArray& vertex_array, const Sim& sim)
{
    // Terrain
    drawTerrain(
        vertex_array,
        sim.terrain);
    drawPointCloud(
        vertex_array,
        sim.terrain.landmarks,
        MarkerType::SQUARE,
        0.2,
        sf::Color(200, 100, 200));

    // Robot
    drawPose(
        vertex_array,
        sim.robot.pose,
        0.2,
        sf::Color::Red);

    // Ranges in true frame
    drawPointCloud(
        vertex_array,
        sim.sensor.ranges,
        MarkerType::CIRCLE,
        0.05,
        sf::Color::Blue,
        sim.robot.pose);
}

void drawStateEstimator(
    sf::VertexArray& vertex_array,
    const StateEstimator& state_estimator)
{
    // Implementation specific rendering

    if (const StateEstimatorEkf* ekf = dynamic_cast<const StateEstimatorEkf*>(&state_estimator)) {
        drawStateEstimatorEkf(vertex_array, *ekf);
    } else if (const StateEstimatorMht* mht = dynamic_cast<const StateEstimatorMht*>(&state_estimator)) {
        drawStateEstimatorMht(vertex_array, *mht);
    } else {
        // Not implemented
    }

    // State estimate
    drawPose(
        vertex_array,
        state_estimator.getStateEstimate(),
        0.2,
        sf::Color::Black);
}

void drawController(sf::VertexArray& vertex_array, const Controller& controller)
{
    // No implementation specific rendering

    // Target
    drawTarget(
        vertex_array,
        controller.getTarget(),
        0.2,
        sf::Color::Black);
}


void Renderer::render()
{
    window.clear(sf::Color::White);
    vertex_array.clear();

    sf::View view;
    view.setCenter(camera.position.x(), camera.position.y());
    view.setSize((double)window.getSize().x, -(double)window.getSize().y); // Flip y axis
    view.zoom(1.0/camera.scale);
    window.setView(view);

    drawSim(vertex_array, state.sim);
    drawStateEstimator(vertex_array, state.state_estimator);
    drawController(vertex_array, state.controller);

    window.draw(vertex_array);
    window.display();
}
