
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
            transformPoint(pose, point.pos()),
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

            Eigen::Vector2d position = transformPoint(pose, point_cloud.points[i].pos());

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

void drawStateEstimatorEKF(
    sf::RenderWindow& window,
    const StateEstimatorEKF& state_estimator)
{
    drawPoseCovariance(
        window,
        state_estimator.x.pose,
        state_estimator.x.covariance);
    drawPose(
        window,
        state_estimator.x.pose,
        0.2,
        sf::Color::Black);
}

void drawStateEstimator(
    sf::RenderWindow& window,
    const StateEstimator& state_estimator)
{
    const StateEstimatorEKF* ekf = dynamic_cast<const StateEstimatorEKF*>(&state_estimator);
    if (ekf) {
        drawStateEstimatorEKF(window, *ekf);
        return;
    }

    // Not implemented
}

void drawState(sf::RenderWindow& window, const State& state)
{
    // Terrain
    drawTerrain(
        window,
        state.terrain);
    drawPointCloud(
        window,
        state.terrain.landmarks,
        MarkerType::SQUARE,
        0.2,
        sf::Color(200, 100, 200));

    // State estimate
    drawStateEstimator(window, state.state_estimator);

    // Target
    drawTarget(
        window,
        state.target,
        0.2,
        sf::Color::Black);

    // Robot
    drawPose(
        window,
        state.robot.pose,
        0.2,
        sf::Color::Red);

    // Sensor information in true frame
    drawPointCloud(
        window,
        state.lidar.ranges,
        MarkerType::CIRCLE,
        0.05,
        sf::Color::Blue,
        state.robot.pose);
    drawPointCloud(
        window,
        state.lidar.features,
        MarkerType::RING,
        0.4,
        sf::Color::Magenta,
        state.robot.pose);

    // Sensor information in state estimator frame
    drawPointCloud(
        window,
        state.lidar.features,
        MarkerType::RING,
        0.4,
        sf::Color::Green,
        state.state_estimator.getStateEstimate());
}

void Renderer::render()
{
    window.clear(sf::Color::White);

    sf::View view;
    view.setCenter(camera.position.x(), camera.position.y());
    view.setSize((double)window.getSize().x, -(double)window.getSize().y); // Flip y axis
    view.zoom(1.0/camera.scale);
    window.setView(view);

    drawState(window, state);

    window.display();
}
