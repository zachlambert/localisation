
#include "state/terrain.h"
#include <stack>
#include "utils/render_utils.h"


// ====== Terrain =====

void Terrain::addElementLandmarks(const Element& element, size_t descriptor_size)
{
    for (const auto& vertex: element.vertices) {
        Eigen::VectorXd pos = element.pos + vertex;
        landmarks.points.push_back(Point(pos, randomDescriptor(descriptor_size)));
    }
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
    Eigen::Vector2d direction = pose.rotation() * getDirection(angle);

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


// ===== Other functions =====

void createTerrain(Terrain& terrain)
{
    size_t descriptor_size = 8;

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
        terrain.elements.push_back(element);
    }
    {
        Terrain::Element element;
        element.addVertex(-x2, y1);
        element.addVertex(-x2, y2);
        element.addVertex(x2, y2);
        element.addVertex(x2, y1);
        terrain.elements.push_back(element);
    }
    {
        Terrain::Element element;
        element.addVertex(x1, -y2);
        element.addVertex(x1, y2);
        element.addVertex(x2, y2);
        element.addVertex(x2, -y2);
        terrain.elements.push_back(element);
    }
    {
        Terrain::Element element;
        element.addVertex(-x2, -y1);
        element.addVertex(x2, -y1);
        element.addVertex(x2, -y2);
        element.addVertex(-x2, -y2);
        terrain.elements.push_back(element);
    }

    // Add an arbitrary element within
    {
        Terrain::Element element;
        element.addVertex(-1, -1);
        element.addVertex(-1, -1);
        element.addVertex(-1.5, 1);
        element.addVertex(0, 1.2);
        element.addVertex(1.5, 0.8);
        element.addVertex(1.1, -1.2);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element, descriptor_size);
    }

    // Add detail to inner walls
    { // Left
        Terrain::Element element;
        element.addVertex(-x1, y1);
        element.addVertex(-x1+0.1, 0.5*y1);
        element.addVertex(-x1+0.15, 0);
        element.addVertex(-x1+0.15, -0.5*y1);
        element.addVertex(-x1, -y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element, descriptor_size);
    }
    { // Right
        Terrain::Element element;
        element.addVertex(x1, -y1);
        element.addVertex(x1-0.08, -0.5*y1);
        element.addVertex(x1-0.05, 0.1*y1);
        element.addVertex(x1-0.2, 0.7*y1);
        element.addVertex(x1, y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element, descriptor_size);
    }
    { // Top
        Terrain::Element element;
        element.addVertex(x1, y1);
        element.addVertex(0.8*x1, y1-0.1);
        element.addVertex(0.1*x1, y1-0.2);
        element.addVertex(-0.4*x1, y1-0.1);
        element.addVertex(-x1, y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element, descriptor_size);
    }
    { // Bottom
        Terrain::Element element;
        element.addVertex(-x1, -y1);
        element.addVertex(-0.4*x1, -y1+0.1);
        element.addVertex(0*x1, -y1+0.2);
        element.addVertex(0.5*x1, -y1+0.1);
        element.addVertex(x1, -y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element, descriptor_size);
    }
}
