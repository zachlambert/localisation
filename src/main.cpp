#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "sim.h"

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
    Camera camera;
    Sim sim;

    bool running = false;
    sf::Clock clock;

    const double fixed_dt = 0.1;

    while (window.isOpen()) {
        double dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
            if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::Q) {
                    if (!running) {
                        sim.stepAll();
                    }
                    running = true;
                }
                if (event.key.code == sf::Keyboard::W) {
                    if (running) {
                        running = false;
                        sim.start(fixed_dt);
                    } else {
                        if (sim.step()) {
                            sim.start(fixed_dt);
                        }
                    }
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mouse_pos(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mapped_pos = window.mapPixelToCoords(mouse_pos);
                    sim.target.pose.position().x() = mapped_pos.x;
                    sim.target.pose.position().y() = mapped_pos.y;
                }
            }
        }

        if (running) {
            sim.start(dt);
            sim.stepAll();
        }

        window.clear(sf::Color::White);

        sf::View view;
        view.setCenter(camera.position.x(), camera.position.y());
        view.setSize((double)window.getSize().x, -(double)window.getSize().y); // Flip y axis
        view.zoom(1.0/camera.scale);
        window.setView(view);

        window.draw(sim);

        window.display();
    }

    return 0;
}
