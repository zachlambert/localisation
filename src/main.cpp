#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "renderer.h"
#include "robot.h"
#include "terrain.h"

void create_terrain(Terrain& terrain)
{
    // Create bounding box, with overlapping edges at the corners for simplicity
    double inner_width = 8;
    double inner_height = 8;
    double outer_thickness = 1;
    double x1 = inner_width/2;
    double x2 = inner_width/2 + outer_thickness;
    double y1 = inner_height/2;
    double y2 = inner_height/2 + outer_thickness;
    {
        Terrain::Element element;
        element.add_vertex(-x1, -y2);
        element.add_vertex(-x2, -y2);
        element.add_vertex(-x2, y2);
        element.add_vertex(-x1, y2);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(-x2, y1);
        element.add_vertex(-x2, y2);
        element.add_vertex(x2, y2);
        element.add_vertex(x2, y1);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(x1, -y2);
        element.add_vertex(x1, y2);
        element.add_vertex(x2, y2);
        element.add_vertex(x2, -y2);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(-x2, -y1);
        element.add_vertex(x2, -y1);
        element.add_vertex(x2, -y2);
        element.add_vertex(-x2, -y2);
        terrain.add_element(element);
    }

    // Add an arbitrary element within
    {
        Terrain::Element element;
        element.add_vertex(-1, -1);
        element.add_vertex(-1.5, 1);
        element.add_vertex(0, 1.2);
        element.add_vertex(1.5, 0.8);
        element.add_vertex(1.1, -1.2);
        terrain.add_element(element);
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Localistaion");

    Camera camera(0, 0, 80);
    Renderer renderer(window);

    Robot robot(0.1, sf::Color::Red);
    robot.pose.position().x() = -3;

    Terrain terrain(sf::Color::Black);
    create_terrain(terrain);
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
