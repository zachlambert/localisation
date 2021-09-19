#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "geometry.h"
#include "render_objects.h"
#include "robot.h"
#include "sensor.h"
#include "terrain.h"

struct Camera {
    Eigen::Vector2d position;
    double scale;
    Camera():
        position(0, 0),
        scale(80)
    {}
};

int main()
{
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Localistaion");

    Terrain terrain;
    create_terrain(terrain);

    Camera camera;

    Robot robot;
    robot.pose.position() = Eigen::Vector2d(-3, 3);
    robot.pose.orientation() = 0.5;
    robot.vel.linear() = Eigen::Vector2d(1, 0.2);
    robot.vel.angular() = 1;

    LaserScan scan;
    scan.setNumPoints(100);
    scan.pose = robot.pose;
    scan.sample(terrain);

    Target target;
    target.position = Eigen::Vector2d(-3, 0);

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
            if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::Return) {
                    robot.step_model(target.position, 0.1);
                    scan.pose = robot.pose;
                    scan.sample(terrain);
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mouse_pos(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mapped_pos = window.mapPixelToCoords(mouse_pos);
                    target.position.x() = mapped_pos.x;
                    target.position.y() = mapped_pos.y;
                }
            }
        }

        window.clear(sf::Color::White);

        sf::View view;
        view.setCenter(camera.position.x(), camera.position.y());
        view.setSize((double)window.getSize().x, -(double)window.getSize().y); // Flip y axis
        view.zoom(1.0/camera.scale);
        window.setView(view);

        // Draw objects
        window.draw(terrain);
        window.draw(robot);
        window.draw(target);
        window.draw(scan);

        window.display();
    }

    return 0;
}
