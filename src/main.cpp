#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "state/state.h"
#include "utils/renderer.h"

int main()
{
    // Configure models and the state estimator
    MotionModel motion_model;
    {
        MotionModel::Config config;
        config.var_weights.d_d = 0.1;
        config.var_weights.phi1_phi1 = 0.1;
        config.var_weights.phi2_phi2 = 0.1;
        motion_model.setConfig(config);
    }

    StateEstimatorEKF state_estimator(motion_model);

    Controller controller;

    State state(motion_model, state_estimator, controller);

    Camera camera;
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Localisation");
    Renderer renderer(state, camera, window);

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
                        state.stepRemaining();
                    }
                    running = true;
                }
                if (event.key.code == sf::Keyboard::W) {
                    if (running) {
                        running = false;
                        state.start(fixed_dt);
                    } else {
                        if (state.step()) {
                            state.start(fixed_dt);
                        }
                    }
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mouse_pos(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mapped_pos = window.mapPixelToCoords(mouse_pos);
                    state.target.position().x() = mapped_pos.x;
                    state.target.position().y() = mapped_pos.y;
                }
            }
        }

        if (running) {
            state.start(dt);
            state.stepRemaining();
        }

        renderer.render();
    }

    return 0;
}

