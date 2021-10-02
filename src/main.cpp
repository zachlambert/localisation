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

    RangeModel range_model;
    {
        RangeModel::Config config;
        config.max_range = 10;
        config.hit_var = std::pow(0.05, 2);
        config.short_scale = 1;

        config.prior_hit = 0.85;
        config.prior_invalid = 0.05;
        config.prior_random = 0.05;
        config.prior_short = 0.05;

        range_model.setConfig(config);
    }

    FeatureModel feature_model;
    {
        FeatureModel::Config config;
        config.range_var = std::pow(0.1, 2);
        config.angle_var = std::pow(0.01, 2);
        config.descriptor_var = std::pow(0.05, 2);

        config.correspondance_p_threshold = 0; // TODO: Increase a bit
        config.false_negative_p = 0.1;
        config.false_positive_rate = 4;

        feature_model.setConfig(config);
    }

    StateEstimatorEKF state_estimator(motion_model);

    Controller controller;

    State state(
        motion_model,
        range_model,
        feature_model,
        state_estimator,
        controller);

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

