#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "renderer.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Localistaion");

    Camera camera(0, 0, 100);
    Renderer renderer(window);

    Pose robot_pose;
    robot_pose.position().y() = 2;
    robot_pose.orientation() = 0.5;
    sf::RectangleShape robot_rect(sf::Vector2f(1, 5));
    robot_rect.setOrigin(0.5, 2.5);
    robot_rect.setFillColor(sf::Color::Red);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
        }
        renderer.add_command(robot_pose, robot_rect);
        renderer.render(camera);
    }

    return 0;
}
