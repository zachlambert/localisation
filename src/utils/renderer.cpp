
#include "utils/renderer.h"
#include "utils/render_utils.h"


void drawPoseCovariance(
    sf::RenderWindow& window,
    const Pose& pose,
    const Eigen::Matrix3d& cov)
{
    const double cov_position_scaling = 40;
    const double cov_orientation_scaling = 10;

    sf::VertexArray vertex_array;
    vertex_array.setPrimitiveType(sf::Triangles);

    addCovarianceEllipse(
        vertex_array,
        cov.block<2,2>(0,0),
        cov_position_scaling,
        sf::Color::Green);
    addSegment(
        vertex_array,
        0.5,
        0,
        cov_orientation_scaling * cov(2,2),
        sf::Color::Cyan);

    window.draw(vertex_array, getRenderTransform(pose));
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
    drawPoseCovariance(
        window,
        state.state_estimator.pose,
        state.state_estimator.covariance);
    drawPose(
        window,
        state.state_estimator.pose,
        0.2,
        sf::Color::Black);

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
        state.lidar.scan,
        MarkerType::CIRCLE,
        0.05,
        sf::Color::Blue,
        state.robot.pose);
    drawPointCloud(
        window,
        state.lidar.landmarks,
        MarkerType::RING,
        0.4,
        sf::Color::Magenta,
        state.robot.pose);

    // Sensor information in state estimator frame
    drawPointCloud(
        window,
        state.lidar.landmarks,
        MarkerType::RING,
        0.4,
        sf::Color::Green,
        state.state_estimator.pose);
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