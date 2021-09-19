#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "renderer.h"
#include "robot.h"
#include "sensor.h"
#include "terrain.h"
#include "render_utils.h"

struct World {

};

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

    Pose target;
    target.position() = Eigen::Vector2d(-3, 3);
    Marker target_marker;
    target_marker.setSize(1);
    target_marker.setColor(sf::Color::Red);

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
                    robot.step_model(target, 0.1);
                    scan.sample(robot.pose, terrain);
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f screen_size = window.getView().getSize();
                    sf::Vector2f mouse_pos(
                        event.mouseButton.x - screen_size.x/2,
                        event.mouseButton.y - screen_size.y/2);
                    mouse_pos = camera.get_transform().getInverse()
                        .transformPoint(mouse_pos);

                    target.position().x() = mouse_pos.x;
                    target.position().y() = mouse_pos.y;
                }
            }
        }

        renderer.add_command(robot.pose, robot.to_render.body);
        renderer.add_command(robot.pose, robot.to_render.direction);
        renderer.add_command(terrain.pose, terrain.to_render.terrain);
        renderer.add_command(Pose(), scan.to_render.measurements);

        // target_marker.setPosition(target.position().x(), target.position().y());
        
        window.clear(sf::Color::White);
        target_marker.setPosition(0, 0);
        sf::View view;
        view.setCenter(0, 0);
        view.setSize(window.getSize().x, window.getSize().y);
        view.zoom(1.0/100);
        window.setView(view);
        window.draw(target_marker);
        window.display();

        // renderer.render(camera);
    }

    return 0;
}
