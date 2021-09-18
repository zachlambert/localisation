#ifndef RENDERER_H
#define RENDERER_H

#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "geometry.h"

struct Camera {
    Pose pose;
    double scale;

    Camera(): pose(), scale(1) {}
    Camera(double x, double y, double s)
    {
        pose.position().x() = x;
        pose.position().y() = y;
        scale = s;
    }

    sf::Transform get_transform()const
    {
        sf::Transform transform(sf::Transform::Identity);
        // Opposite order to expected
        transform.translate(pose.position().x(), pose.position().y());
        transform.rotate(pose.orientation());
        transform.scale(scale, -scale);
        return transform;
    }
};

class Renderer{
public:
    Renderer(sf::RenderWindow& window, size_t MAX_COMMANDS=128):
        window(window)
    {
        commands.reserve(MAX_COMMANDS);
    }

    void add_command(const Pose& pose, const sf::RectangleShape& data)
    {
        commands.push_back(Command(
            pose, Command::RECT, (const void*)&data
        ));
    }
    
    void render(const Camera& camera)
    {
        // Make the window adapt to changes in size, and centre on (0, 0)
        sf::Vector2u window_size = window.getSize();
        sf::View view;
        view.setCenter(0, 0);
        view.setSize(window_size.x, window_size.y);
        window.setView(view);

        window.clear(sf::Color::White);
        sf::Transform view_transform = camera.get_transform();
        sf::Transform transform;
        for (const auto& command: commands) {
            transform = view_transform * command.transform;
            switch(command.type) {
                case Command::RECT:
                    window.draw(*(const sf::RectangleShape*)command.data, transform);
                    break;
                default:
                    break;
            }
        }
        window.display();
    }

private:
    struct Command {
        sf::Transform transform;
        enum Type {
            LINE,
            CIRCE,
            RECT,
            IMAGE
        } type;
        const void* data;

        Command(const Pose& pose, Command::Type type, const void* data):
            transform(sf::Transform::Identity), type(type), data(data)
        {
            // Opposite order to expected
            transform.translate(pose.position().x(), pose.position().y());
            transform.rotate(pose.orientation());
        }
    };

    sf::RenderWindow& window;
    std::vector<Command> commands;
};


#endif
