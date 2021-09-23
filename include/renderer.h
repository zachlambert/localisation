#ifndef RENDERER_H
#define RENDERER_H

#include "state.h"
#include <SFML/Graphics.hpp>

struct Camera {
    Eigen::Vector2d position;
    double scale;
    Camera():
        position(0, 0),
        scale(80)
    {}
};


class Renderer {
public:
    Renderer(const State& state, Camera &camera, sf::RenderWindow& window):
        state(state),
        camera(camera),
        window(window)
    {}

    void render();

private:
    const State& state;
    Camera& camera;
    sf::RenderWindow& window;
};

#endif
