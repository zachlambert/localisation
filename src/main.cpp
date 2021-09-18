#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "renderer.h"
#include "terrain.h"

struct Robot {
    Pose pose;

    struct {
        sf::CircleShape body;
        sf::RectangleShape direction;
    } to_render;

    Robot(double radius, sf::Color color): pose()
    {
        to_render.body.setRadius(radius);
        to_render.body.setFillColor(color);
        to_render.body.setOrigin(radius, radius);

        to_render.direction.setSize(sf::Vector2f(radius*3, radius*0.5));
        to_render.direction.setOrigin(0, to_render.direction.getSize().y/2);
        to_render.direction.setFillColor(color);
    }
};

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Localistaion");

    Camera camera(0, 0, 100);
    Renderer renderer(window);

    Robot robot(0.1, sf::Color::Red);

    Terrain terrain(sf::Color::Black);
    {
        Terrain::Element element;
        element.pos.x() = 0;
        element.pos.y() = 0;
        element.add_vertex(0, 0);
        element.add_vertex(0, 1);
        element.add_vertex(3, 1);
        element.add_vertex(3, -4);
        element.add_vertex(2, 0);
        terrain.add_element(element);
    }
    terrain.initialise();

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
        }
        renderer.add_command(robot.pose, robot.to_render.body);
        renderer.add_command(robot.pose, robot.to_render.direction);
        renderer.add_command(terrain.pose, terrain.to_render.terrain);
        renderer.render(camera);
    }

    return 0;
}
