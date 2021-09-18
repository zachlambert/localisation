#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "renderer.h"
#include "robot.h"
#include "sensor.h"
#include "terrain.h"


int main()
{
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Localistaion");

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

    Eigen::Vector2d target(-3, 3);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
            if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::Return) {
                    robot.step_model(target, 0.1);
                    scan.sample(robot.pose, terrain);
                }
            }
        }
        renderer.add_command(robot.pose, robot.to_render.body);
        renderer.add_command(robot.pose, robot.to_render.direction);
        renderer.add_command(terrain.pose, terrain.to_render.terrain);
        renderer.add_command(robot.pose, scan.to_render.measurements);
        renderer.render(camera);
    }

    return 0;
}
