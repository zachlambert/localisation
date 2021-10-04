#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "state/state.h"
#include "utils/renderer.h"

int main()
{
    // ===== Configure models =====

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
        config.range_var = std::pow(0.5, 2);
        config.angle_var = std::pow(0.1, 2);
        config.descriptor_var = std::pow(0.3, 2);

        config.false_negative_p = 0.2;
        config.false_positive_rate = 7;

        feature_model.setConfig(config);
    }

    // ===== Configure simulation =====

    Sim sim(motion_model, range_model);
    {
        createTerrain(sim.terrain);

        sim.robot.pose.position() = Eigen::Vector2d(-3, 3);
        sim.robot.pose.orientation() = 0.5;

        sim.range_sensor.setScanSize(100);
    }

    // ===== Configure algorithms =====

    FeatureDetectorFake feature_detector(sim.terrain, sim.robot, feature_model);

    FeatureMatcher feature_matcher(feature_model);
    {
        FeatureMatcher::Config config;
        config.correspondance_p_threshold = 10; // probability density
        config.use_feature_model = true;
        feature_matcher.setConfig(config);
    }

    StateEstimatorEKF state_estimator(
        motion_model,
        range_model,
        feature_model,
        feature_detector,
        feature_matcher
    );
    {
        state_estimator.resetEstimate(sim.robot.pose);
    }

    Controller controller;
    {
        Pose initial_target;
        initial_target.position() = Eigen::Vector2d(2, 2);
        initial_target.orientation() = 0;
        controller.setTarget(initial_target);
    }

    // ===== Initialise program state (sim + algorithms) =====

    State state(
        sim,
        state_estimator,
        controller
    );

    // ===== Create a window and start runnig =====

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
                else if (event.key.code == sf::Keyboard::W) {
                    if (running) {
                        running = false;
                        state.start(fixed_dt);
                    } else {
                        if (state.step()) {
                            state.start(fixed_dt);
                        }
                    }
                }
                else if (event.key.code == sf::Keyboard::E) {
                    state_estimator.resetEstimate(sim.robot.pose);
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mouse_pos(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mapped_pos = window.mapPixelToCoords(mouse_pos);

                    Pose target;
                    target.position().x() = mapped_pos.x;
                    target.position().y() = mapped_pos.y;
                    state.controller.setTarget(target);
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

