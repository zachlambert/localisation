#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

// #include "renderer.h"
// #include "robot.h"
// #include "sensor.h"
// #include "terrain.h"
#include "render_objects.h"
#include "geometry.h"

struct Camera {
    Eigen::Vector2d position;
    double scale;
    Camera():
        position(0, 0),
        scale(50)
    {}
};

class Robot: public sf::Drawable {
public:
    Robot() {}

    // Data
    Pose pose;

    // Render objects
    mutable struct {
        sf::PoseMarker pose_marker;
    } to_draw;

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        to_draw.pose_marker.setPosition(pose.position().x(), pose.position().y());
        to_draw.pose_marker.setRotation(pose.orientation());
        target.draw(to_draw.pose_marker, states);
    }
};

int main()
{
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Localistaion");

    /*
    Camera camera(0, 0, 80);
    Renderer renderer(window);

    Robot robot(0.1, sf::Color::Red);
    robot.pose.position().x() = -3;
    robot.pose.position().y() = -3;
    robot.pose.orientation() = 2;

    Terrain terrain(sf::Color(150, 150, 150));
    create_terrain(terrain);
    terrain.initialise();

    LaserScan scan(100);
    scan.sample(robot.pose, terrain);
    */

    Camera camera;

    Robot robot;

    // Target target;
    // target.position = Eigen::Vector2d(-3, 3);
    // target.marker.setSize(1);
    // target.marker.setColor(sf::Color::Green);

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
            if (event.type == sf::Event::KeyReleased) {
                /*
                if (event.key.code == sf::Keyboard::Return) {
                    robot.step_model(target.position, 0.1);
                    scan.sample(robot.pose, terrain);
                }
                */
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mouse_pos(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mapped_pos = window.mapPixelToCoords(mouse_pos);
                    // target.position.x() = mapped_pos.x;
                    // target.position.y() = mapped_pos.y;
                }
            }
        }

        window.clear(sf::Color::White);

        sf::View view;
        view.setCenter(camera.position.x(), camera.position.y());
        view.setSize(window.getSize().x, window.getSize().y);
        view.zoom(1.0/camera.scale);
        window.setView(view);

        // Draw objects
        window.draw(robot);

        window.display();
    }

    return 0;
}
