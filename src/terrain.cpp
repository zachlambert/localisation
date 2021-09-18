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

double Terrain::query_intersection(const Pose& pose, double angle)const
{
    // For now, just using simple method.
    // If it becomes a bottleneck, implement a quadtree
    // TODO
    return 3; // Arbitrary value for visualisation
}
