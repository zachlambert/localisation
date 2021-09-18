#include "terrain.h"

#include <iostream>
#include <stack>

#include "render_utils.h"

void Terrain::initialise()
{
    // Create vertex array
    to_render.terrain.setPrimitiveType(sf::Triangles);
    for (const auto &element: elements) {
        Pose element_pose = pose;
        element_pose.position() += element.pos;
        add_mesh(to_render.terrain, element_pose, element.vertices, color);
    }
}

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

double Terrain::query_intersection(const Pose& pose, double angle)const
{
    // For now, just using simple method.
    // If it becomes a bottleneck, implement a quadtree

    Eigen::Vector2d origin = pose.position();
    Eigen::Vector2d direction = pose.get_R() * get_direction(angle);

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

void create_terrain(Terrain& terrain)
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
        element.add_vertex(-x1, -y2);
        element.add_vertex(-x2, -y2);
        element.add_vertex(-x2, y2);
        element.add_vertex(-x1, y2);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(-x2, y1);
        element.add_vertex(-x2, y2);
        element.add_vertex(x2, y2);
        element.add_vertex(x2, y1);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(x1, -y2);
        element.add_vertex(x1, y2);
        element.add_vertex(x2, y2);
        element.add_vertex(x2, -y2);
        terrain.add_element(element);
    }
    {
        Terrain::Element element;
        element.add_vertex(-x2, -y1);
        element.add_vertex(x2, -y1);
        element.add_vertex(x2, -y2);
        element.add_vertex(-x2, -y2);
        terrain.add_element(element);
    }

    // Add an arbitrary element within
    {
        Terrain::Element element;
        element.add_vertex(-1, -1);
        element.add_vertex(-1.5, 1);
        element.add_vertex(0, 1.2);
        element.add_vertex(1.5, 0.8);
        element.add_vertex(1.1, -1.2);
        terrain.add_element(element);
    }
}
