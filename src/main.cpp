#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Localistaion");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
        }
        window.clear(sf::Color::White);
        sf::RectangleShape rect(sf::Vector2f(100, 10));
        rect.setPosition(100, 100);
        rect.setFillColor(sf::Color::Red);
        window.draw(rect);
        window.display();
    }

    return 0;
}
