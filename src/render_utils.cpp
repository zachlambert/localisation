
#include "render_utils.h"

#include <stack>
#include <iostream>

#include "geometry.h"


sf::Vertex create_vertex(sf::Color color, const Eigen::Vector2d& pos)
{
    sf::Vertex vertex;
    vertex.position.x = pos.x();
    vertex.position.y = pos.y();
    vertex.color = color;
    return vertex;
}

sf::Vertex create_vertex(sf::Color color, double x, double y)
{
    sf::Vertex vertex;
    vertex.position.x = x;
    vertex.position.y = y;
    vertex.color = color;
    return vertex;
}

namespace mesh_triangulation {

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

} // namespace mesh_triangulation

void add_mesh(
    sf::VertexArray& vertex_array,
    const Pose& pose,
    const std::vector<Eigen::Vector2d>& vertices,
    sf::Color color)
{
    // Assumes the vertices to be provided in a clockwise order.
    // Works on any mesh
    using namespace mesh_triangulation;

    std::vector<size_t> initial(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++) {
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
            bool current_valid = triangle_valid(current[a], current[b], current[c], vertices);
            if (current_valid) {
                vertex_array.append(create_vertex(color, transform_point(pose, vertices[current[a]])));
                vertex_array.append(create_vertex(color, transform_point(pose, vertices[current[b]])));
                vertex_array.append(create_vertex(color, transform_point(pose, vertices[current[c]])));

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

void add_marker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    sf::Color color)
{
    double t = (size > 0.3 ? 0.3 : size) * 0.2; // Thickness
    double d1 = t / std::sqrt(2);
    double d3 = d1 + ((size-t)/2) / std::sqrt(2);
    double d2 = d3 - t / std::sqrt(2);

    std::vector<Eigen::Vector2d> vertices;
    vertices.push_back(Eigen::Vector2d(-d1, 0));
    vertices.push_back(Eigen::Vector2d(-d3, d2));
    vertices.push_back(Eigen::Vector2d(-d2, d3));
    vertices.push_back(Eigen::Vector2d(0, d1));
    vertices.push_back(Eigen::Vector2d(d2, d3));
    vertices.push_back(Eigen::Vector2d(d3, d2));
    vertices.push_back(Eigen::Vector2d(d1, 0));
    vertices.push_back(Eigen::Vector2d(d3, -d2));
    vertices.push_back(Eigen::Vector2d(d2, -d3));
    vertices.push_back(Eigen::Vector2d(0, -d1));
    vertices.push_back(Eigen::Vector2d(-d2, -d3));
    vertices.push_back(Eigen::Vector2d(-d3, -d2));

    Pose pose;
    pose.position() = pos;
    add_mesh(vertex_array, pose, vertices, color);
}

void add_circle_mesh(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    sf::Color color,
    size_t n)
{
    for (size_t i = 0; i < n; i++) {
        vertex_array.append(create_vertex(color, pos));
        Eigen::Vector2d p1 = pos + radius*get_direction(i*2*M_PI/n);
        Eigen::Vector2d p2 = pos + radius*get_direction(((i+1)%n)*2*M_PI/n);
        vertex_array.append(create_vertex(color, p2));
        vertex_array.append(create_vertex(color, p1));
    }
}

