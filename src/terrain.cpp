#include "terrain.h"

#include <iostream>
#include <stack>

// ------ MESH TRIANGULATION -----

static bool vertex_on_the_left(const Eigen::Vector2d& first, const Eigen::Vector2d& second, const Eigen::Vector2d& query)
{
    Eigen::Vector2d dif = second - first;
    Eigen::Vector2d perp = get_S(1)*dif;
    // Perp is to the left of the first displacmenet
    // A positive component along this will indicate the query point is to the left
    dif = query - second;
    double component = perp.dot(dif);
    return (component > 0);
}

static bool triangle_valid(size_t i, size_t j, size_t k, const std::vector<Eigen::Vector2d>& vertices)
{
    // 1. Triangle is clockwise
    if (vertex_on_the_left(vertices[i], vertices[j], vertices[k])) return false;

    // 2. No self-intersection
    for (size_t q = j+1; q < vertices.size(); q++) {
        if (q==k) continue;
        if (vertex_on_the_left(vertices[i], vertices[j], vertices[q])) continue;
        if (vertex_on_the_left(vertices[j], vertices[k], vertices[q])) continue;
        if (vertex_on_the_left(vertices[k], vertices[i], vertices[q])) continue;
        return false;
    }

    return true;
}

sf::Vertex create_vertex(sf::Color color, const Eigen::Vector2d& pos)
{
    sf::Vertex vertex;
    vertex.position.x = pos.x();
    vertex.position.y = pos.y();
    vertex.color = color;
    return vertex;
}

static void load_element(sf::Color color, const Terrain::Element& element, sf::VertexArray& vertex_array)
{
    // Assumes the vertices to be provided in a clockwise order.
    // Works on any mesh

    std::vector<size_t> initial(element.vertices.size());
    for (size_t i = 0; i < element.vertices.size(); i++) {
        initial[i] = i;
    }

    std::stack<std::vector<size_t>> stack;
    stack.push(initial);

    int a, b, c;
    while (!stack.empty()) {
        const std::vector<size_t> current = stack.top();
        stack.pop();
        a = 0;
        b = 1;
        c = 2;
        bool valid = true;
        while (c!= current.size()) {
            bool current_valid = triangle_valid(current[a], current[b], current[c], element.vertices);
            if (current_valid) {
                vertex_array.append(create_vertex(color, element.vertices[current[a]]));
                vertex_array.append(create_vertex(color, element.vertices[current[b]]));
                vertex_array.append(create_vertex(color, element.vertices[current[c]]));

                if (!valid) {
                    valid = true;
                    int new_start = b;
                    int new_end = c;
                    std::vector<size_t> new_indices(c-b+1);
                    for (int i = 0; i < c-b+1; i++) {
                        new_indices[i] = current[b+i];
                    }
                    stack.push(new_indices);
                }
                b = c;
            } else {
                valid = false;
            }
            c++;
        }
        if (!valid) {
            int new_start = b;
            int new_end = c-1;
            std::vector<size_t> new_indices(c-b+1);
            for (int i = 0; i < c-b+1; i++) {
                new_indices[i] = current[b+i];
            }
            stack.push(new_indices);
        }
    }
}

void Terrain::initialise()
{
    // Create vertex array
    to_render.terrain.setPrimitiveType(sf::Triangles);
    for (const auto &element: elements) {
        load_element(color, element, to_render.terrain);
    }
}

double Terrain::query_intersection(const Pose& pose, double angle)
{
    // For now, just using simple method.
    // If it becomes a bottleneck, implement a quadtree
    // TODO
    return 0;
}
