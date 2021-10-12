
#include "utils/renderer.h"
#include "utils/render_utils.h"


void drawPoseCovariance(
    sf::RenderWindow& window,
    const Pose& pose,
    const Eigen::Matrix3d& cov)
{
    const double std_position_scaling = 3;
    const double std_orientation_scaling = 3;

    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);

    addCovarianceEllipse(
        vertex_array,
        cov.block<2,2>(0,0),
        std_position_scaling,
        sf::Color::Green);
    addSegment(
        vertex_array,
        Eigen::Vector2d(0, 0),
        pose.orientation(),
        0.5,
        std_orientation_scaling * std::sqrt(cov(2,2)),
        sf::Color::Cyan);

    sf::Transform transform;
    transform.translate(pose.position().x(), pose.position().y());
    window.draw(vertex_array, transform);
}

void drawPose(
    sf::RenderWindow& window,
    const Pose& pose,
    double size,
    sf::Color color)
{
    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);
    addEllipse(vertex_array, size, size, 0, color);
    addLine(
        vertex_array,
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1.5*size, 0),
        LineType::ARROW,
        color,
        size*0.33);

    window.draw(vertex_array, getRenderTransform(pose));
}

void drawTerrain(
    sf::RenderWindow& window,
    const Terrain& terrain)
{
    const sf::Color color(150, 150, 150);

    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);
    for (const auto& element: terrain.elements) {
        addMesh(
            vertex_array,
            element.pos,
            element.vertices,
            color
        );
    }

    window.draw(vertex_array);
}

void drawPointCloud(
    sf::RenderWindow& window,
    const PointCloud& point_cloud,
    MarkerType marker_type,
    double size,
    sf::Color color,
    const Pose& pose = Pose())
{
    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);
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

    window.draw(vertex_array);
}

void drawPointCloudWithIndicators(
    sf::RenderWindow& window,
    const PointCloud& point_cloud,
    const std::vector<bool>& indicators,
    MarkerType marker_type,
    double size,
    sf::Color color,
    const Pose& pose = Pose())
{
    bool use_indicators = (indicators.size() == point_cloud.points.size());

    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);
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

    window.draw(vertex_array);
}

void drawMatches(
    sf::RenderWindow& window,
    const Pose& pose,
    const FeatureMatcher::Result& match_result,
    sf::Color line_color)
{
    if (!match_result.known_features || !match_result.observed_features) return;

    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);
    for (size_t i = 0; i < match_result.matches.size(); i++) {
        addLine(
            vertex_array,
            match_result.known_features->points[match_result.matches[i].index_known].pos,
            transformPoint(pose, match_result.observed_features->points[match_result.matches[i].index_observed].pos),
            LineType::LINE,
            line_color,
            0.05);
    }

    window.draw(vertex_array);
}

// Target
void drawTarget(
    sf::RenderWindow& window,
    const Pose& pose,
    double size,
    sf::Color color)
{
    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);

    addMarker(
        vertex_array,
        pose.position(),
        MarkerType::CROSS,
        color,
        size);

    window.draw(vertex_array);
}

void drawStateEstimatorEkf(
    sf::RenderWindow& window,
    const StateEstimatorEkf& state_estimator)
{
    drawPoseCovariance(
        window,
        state_estimator.estimate.pose,
        state_estimator.estimate.covariance);

    // Detected features
    drawPointCloud(
        window,
        state_estimator.features,
        MarkerType::RING,
        0.4,
        sf::Color::Green,
        state_estimator.getStateEstimate());
    if (state_estimator.match_result.known_features) {
        drawPointCloudWithIndicators(
            window,
            *state_estimator.match_result.known_features,
            state_estimator.match_result.indicators,
            MarkerType::RING,
            0.4,
            sf::Color::Blue,
            Pose());
    }

    // Correspondances
    drawMatches(
        window,
        state_estimator.getStateEstimate(),
        state_estimator.match_result,
        sf::Color::Black);
}

void drawSim(sf::RenderWindow& window, const Sim& sim)
{
    // Terrain
    drawTerrain(
        window,
        sim.terrain);
    drawPointCloud(
        window,
        sim.terrain.landmarks,
        MarkerType::SQUARE,
        0.2,
        sf::Color(200, 100, 200));

    // Robot
    drawPose(
        window,
        sim.robot.pose,
        0.2,
        sf::Color::Red);

    // Ranges in true frame
    drawPointCloud(
        window,
        sim.sensor.ranges,
        MarkerType::CIRCLE,
        0.05,
        sf::Color::Blue,
        sim.robot.pose);
}

void drawStateEstimator(
    sf::RenderWindow& window,
    const StateEstimator& state_estimator)
{
    // Implementation specific rendering

    if (const StateEstimatorEkf* ekf = dynamic_cast<const StateEstimatorEkf*>(&state_estimator)) {
        drawStateEstimatorEkf(window, *ekf);
    } else {
        // Not implemented
    }

    // State estimate
    drawPose(
        window,
        state_estimator.getStateEstimate(),
        0.2,
        sf::Color::Black);
}

void drawController(sf::RenderWindow& window, const Controller& controller)
{
    // No implementation specific rendering

    // Target
    drawTarget(
        window,
        controller.getTarget(),
        0.2,
        sf::Color::Black);
}


void Renderer::render()
{
    window.clear(sf::Color::White);

    sf::View view;
    view.setCenter(camera.position.x(), camera.position.y());
    view.setSize((double)window.getSize().x, -(double)window.getSize().y); // Flip y axis
    view.zoom(1.0/camera.scale);
    window.setView(view);

    drawSim(window, state.sim);
    drawStateEstimator(window, state.state_estimator);
    drawController(window, state.controller);

    window.display();
}
