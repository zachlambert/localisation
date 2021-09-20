#include "terrain.h"

#include <iostream>
#include <stack>

#include "render_utils.h"


// ====== Terrain =====

Terrain::Terrain()
{
    setColor(sf::Color(150, 150, 150));
    vertex_array.setPrimitiveType(sf::Triangles);
}

void Terrain::addElement(const Element &element)
{
    elements.push_back(element);
    dirty = true;
}

void Terrain::setColor(sf::Color color)
{
    this->color = color;
    dirty = true;
}


// Functions used in query_intersection
namespace intersection {

double line_intersection(
    const Eigen::Vector2d& origin,
    const Eigen::Vector2d& dir,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2)
{
    Eigen::Vector2d perp(-dir.y(), dir.x());
    double h1 = perp.dot(p1 - origin);
    double h2 = perp.dot(p2 - origin);
    if (h1*h2 > 0) return INFINITY;

    double v1 = dir.dot(p1 - origin);
    double v2 = dir.dot(p2 - origin);
    return v1 + (v2 - v1)*(0 - h1) / (h2 - h1);
}

}

double Terrain::queryIntersection(const Pose& pose, double angle)const
{
    using namespace intersection;

    // For now, just using simple method.
    // If it becomes a bottleneck, implement a quadtree

    Eigen::Vector2d origin = pose.position();
    Eigen::Vector2d direction = pose.rotation() * get_direction(angle);

    double min_dist = INFINITY;
    double dist;
    for (const auto& element: elements) {
        for (size_t i = 0; i < element.vertices.size(); i++) {
            dist = line_intersection(
                origin,
                direction,
                element.pos + element.vertices[i],
                element.pos + element.vertices[(i+1)%element.vertices.size()]
            );
            if (dist > 0 && dist < min_dist) {
                min_dist = dist;
            }
        }
    }
    return min_dist;
}


void Terrain::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    for (const auto& element: elements) {
        Pose pose;
        pose.position() = element.pos;
        addMesh(
            vertex_array,
            pose,
            element.vertices,
            color
        );
    }

    dirty = false;
}

void Terrain::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    updateVertices();
    target.draw(vertex_array, states);
}


// ===== Other functions =====

void createTerrain(Terrain& terrain)
{
    // Create bounding box, with overlapping edges at the corners for simplicity
    double inner_width = 8;
    double inner_height = 8;
    double outer_thickness = 1;
    double x1 = inner_width/2;
    double x2 = inner_width/2 + outer_thickness;
    double y1 = inner_height/2;
    double y2 = inner_height/2 + outer_thickness;
    {
        Terrain::Element element;
        element.addVertex(-x1, -y2);
        element.addVertex(-x2, -y2);
        element.addVertex(-x2, y2);
        element.addVertex(-x1, y2);
        terrain.addElement(element);
    }
    {
        Terrain::Element element;
        element.addVertex(-x2, y1);
        element.addVertex(-x2, y2);
        element.addVertex(x2, y2);
        element.addVertex(x2, y1);
        terrain.addElement(element);
    }
    {
        Terrain::Element element;
        element.addVertex(x1, -y2);
        element.addVertex(x1, y2);
        element.addVertex(x2, y2);
        element.addVertex(x2, -y2);
        terrain.addElement(element);
    }
    {
        Terrain::Element element;
        element.addVertex(-x2, -y1);
        element.addVertex(x2, -y1);
        element.addVertex(x2, -y2);
        element.addVertex(-x2, -y2);
        terrain.addElement(element);
    }

    // Add an arbitrary element within
    {
        Terrain::Element element;
        element.addVertex(-1, -1);
        element.addVertex(-1.5, 1);
        element.addVertex(0, 1.2);
        element.addVertex(1.5, 0.8);
        element.addVertex(1.1, -1.2);
        terrain.addElement(element);
    }
}
