
#include "utils/render_utils.h"
#include <stack>
#include <Eigen/Dense>


// General useful functions

sf::Transform getRenderTransform(const Eigen::Vector2d& pos)
{
    sf::Transform transform;
    transform.translate(pos.x(), pos.y());
    return transform;
}


sf::Transform getRenderTransform(const Pose& pose)
{
    sf::Transform transform;
    transform.translate(pose.position().x(), pose.position().y());
    transform.rotate(pose.orientation()*180/M_PI);
    return transform;
}

sf::Vertex createVertex(sf::Color color, const Eigen::Vector2d& pos)
{
    sf::Vertex vertex;
    vertex.position.x = pos.x();
    vertex.position.y = pos.y();
    vertex.color = color;
    return vertex;
}

sf::Vertex createVertex(sf::Color color, double x, double y)
{
    sf::Vertex vertex;
    vertex.position.x = x;
    vertex.position.y = y;
    vertex.color = color;
    return vertex;
}

void addTriangle(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3)
{
    vertex_array.append(createVertex(color, p1));
    vertex_array.append(createVertex(color, p2));
    vertex_array.append(createVertex(color, p3));
}

void addQuad(
    sf::VertexArray& vertex_array,
    sf::Color color,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3,
    const Eigen::Vector2d& p4)
{
    vertex_array.append(createVertex(color, p1));
    vertex_array.append(createVertex(color, p2));
    vertex_array.append(createVertex(color, p3));
    vertex_array.append(createVertex(color, p1));
    vertex_array.append(createVertex(color, p3));
    vertex_array.append(createVertex(color, p4));
}


// ===== addMesh() section =====

namespace mesh_triangulation {

static bool vertexOnTheLeft(const Eigen::Vector2d& first, const Eigen::Vector2d& second, const Eigen::Vector2d& query)
{
    Eigen::Vector2d dif = second - first;
    Eigen::Vector2d perp = crossProductMatrix(1)*dif;
    // Perp is to the left of the first displacmenet
    // A positive component along this will indicate the query point is to the left
    dif = query - second;
    double component = perp.dot(dif);
    return (component > 0);
}

static bool triangleValid(size_t i, size_t j, size_t k, const std::vector<Eigen::Vector2d>& vertices)
{
    // 1. Triangle is clockwise
    if (vertexOnTheLeft(vertices[i], vertices[j], vertices[k])) return false;

    // 2. No self-intersection
    for (size_t q = j+1; q < vertices.size(); q++) {
        if (q==k) continue;
        if (vertexOnTheLeft(vertices[i], vertices[j], vertices[q])) continue;
        if (vertexOnTheLeft(vertices[j], vertices[k], vertices[q])) continue;
        if (vertexOnTheLeft(vertices[k], vertices[i], vertices[q])) continue;
        return false;
    }

    return true;
}

} // namespace mesh_triangulation

void addMesh(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
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
            bool current_valid = triangleValid(current[a], current[b], current[c], vertices);
            if (current_valid) {
                vertex_array.append(createVertex(color, pos + vertices[current[a]]));
                vertex_array.append(createVertex(color, pos + vertices[current[b]]));
                vertex_array.append(createVertex(color, pos + vertices[current[c]]));

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


// ===== addMarker() section =====

namespace markers {

void addCross(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    double thickness,
    sf::Color color)
{
    double d1 = thickness / std::sqrt(2);
    double d3 = d1 + ((size-thickness)/2) / std::sqrt(2);
    double d2 = d3 - thickness / std::sqrt(2);

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

    addMesh(vertex_array, pos, vertices, color);
}

void addCircle(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    sf::Color color,
    size_t n)
{
    for (size_t i = 0; i < n; i++) {
        vertex_array.append(createVertex(color, pos));
        Eigen::Vector2d p1 = pos + radius*getDirection(i*2*M_PI/n);
        Eigen::Vector2d p2 = pos + radius*getDirection(((i+1)%n)*2*M_PI/n);
        vertex_array.append(createVertex(color, p2));
        vertex_array.append(createVertex(color, p1));
    }
}

void addRing(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double radius,
    double thickness,
    sf::Color color,
    size_t n)
{
    if (thickness >= 2*radius) {
        addCircle(vertex_array, pos, radius, color, n);
        return;
    }

    for (size_t i = 0; i < n; i++) {
        double angle = (i*2*M_PI)/n;
        double next = ((i+1)%n)*2*M_PI/n;
        Eigen::Vector2d p[4];
        p[0] = pos + (radius-thickness/2)*getDirection(angle);
        p[1] = pos + (radius-thickness/2)*getDirection(next);
        p[2] = pos + (radius+thickness/2)*getDirection(next);
        p[3] = pos + (radius+thickness/2)*getDirection(angle);
        addQuad(vertex_array, color, p[0], p[1], p[2], p[3]);
    }
}

void addSquare(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    double size,
    sf::Color color)
{
    Eigen::Vector2d points[4];
    for (size_t i = 0; i < 4; i++) {
        points[i] = pos;
        points[i].x() += size/2 * (i/2==0 ? -1 : 1);
        points[i].y() += size/2 * (i%2==0 ? -1 : 1);
    }
    addQuad(
        vertex_array,
        color,
        points[0],
        points[1],
        points[3],
        points[2]);
}

} // namespace marker

void addMarker(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& pos,
    MarkerType type,
    sf::Color color,
    double size)
{
    using namespace markers;

    switch (type) {
        case MarkerType::CROSS:
            addCross(vertex_array, pos, size, size*0.2, color);
            break;
        case MarkerType::CIRCLE:
            addCircle(vertex_array, pos, size/2, color, 32);
            break;
        case MarkerType::RING:
            addRing(vertex_array, pos, size/2, size*0.08, color, 32);
            break;
        case MarkerType::SQUARE:
            addSquare(vertex_array, pos, size, color);
            break;
    }
}


// ===== addLine =====

void addLine(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    LineType type,
    sf::Color color,
    double width)
{
    Eigen::Vector2d n1 = (end - start).normalized();
    Eigen::Vector2d n2 = crossProductMatrix(1) * n1;
    double line_length = (end - start).norm();

    if (type == LineType::ARROW) {
        double head_width = width*2.5;
        double head_length = head_width;
        if (head_length > line_length) head_length = line_length;
        addTriangle(
            vertex_array,
            color,
            end,
            end + n2*head_width/2 - n1*head_length,
            end - n2*head_width/2 - n1*head_length);

        line_length -= head_length;
    }

    if (line_length < 1e-4) return;

    addQuad(
        vertex_array,
        color,
        start + n2*width/2,
        start + n1*line_length + n2*width/2,
        start + n1*line_length - n2*width/2,
        start - n2*width/2);
}


// ===== Other shape =====

void addEllipse(
    sf::VertexArray& vertex_array,
    double width,
    double height,
    double orientation,
    sf::Color color)
{
    Eigen::Vector2d u1 = getDirection(orientation);
    Eigen::Vector2d u2 = crossProductMatrix(1) * u1;
    u1 *= width/2;
    u2 *= height/2;
    auto get_point = [u1, u2](double angle) {
        return u1 * std::cos(angle) + u2 * std::sin(angle);
    };

    constexpr size_t n = 32;
    for (size_t i = 0; i < n; i++) {
        double angle = i*2*M_PI / n;
        double angle2 = ((i+1)%n)*2*M_PI / n;
        addTriangle(vertex_array, color,
            Eigen::Vector2d(0, 0),
            get_point(angle),
            get_point(angle2));
    }
}

void addCovarianceEllipse(
    sf::VertexArray& vertex_array,
    const Eigen::Matrix2d& cov,
    double std_scaling,
    sf::Color color)
{

    Eigen::EigenSolver<Eigen::Matrix2d> eigensolver(cov);
    auto u1 = eigensolver.eigenvectors().col(0).real();
    auto u2 = eigensolver.eigenvectors().col(1).real();
    double v1 = eigensolver.eigenvalues()(0).real();
    double v2 = eigensolver.eigenvalues()(1).real();
    double angle = std::atan2(u1.y(), u1.x());
    double width = std::sqrt(v1) * std_scaling;
    double height = std::sqrt(v2) * std_scaling;
    addEllipse(vertex_array, width, height, angle, color);
}

void addSegment(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& position,
    double orientation,
    double radius,
    double width,
    sf::Color color)
{
    constexpr size_t n = 16;
    const double delta_angle = width / n;
    double angle = orientation - width/2;
    for (size_t i = 0; i < n; i++) {
        addTriangle(vertex_array, color,
            position,
            position + radius * getDirection(angle),
            position + radius * getDirection(angle+delta_angle));
        angle += delta_angle;
    }
}

void addSegmentSlice(
    sf::VertexArray& vertex_array,
    const Eigen::Vector2d& position,
    double orientation,
    double inner_radius,
    double outer_radius,
    double width,
    sf::Color color)
{
    constexpr size_t n = 16;
    const double delta_angle = width / n;
    double angle = orientation - width/2;
    for (size_t i = 0; i < n; i++) {
        addQuad(vertex_array, color,
            position + inner_radius * getDirection(angle),
            position + outer_radius * getDirection(angle),
            position + outer_radius * getDirection(angle+delta_angle),
            position + inner_radius * getDirection(angle+delta_angle));
        angle += delta_angle;
    }
}
