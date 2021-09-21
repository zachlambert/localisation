
#include "point_cloud.h"

#include "render_utils.h"


void PointCloud::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    for (const auto& point: points)
    {
        switch (marker_type) {
            case MarkerType::CROSS:
                addCross(
                    vertex_array,
                    point.pos,
                    marker_size,
                    marker_size*0.2,
                    marker_color);
                break;
            case MarkerType::CIRCLE:
                addCircle(
                    vertex_array,
                    point.pos,
                    marker_size/2,
                    marker_color,
                    12);
                break;
            case MarkerType::RING:
                addRing(
                    vertex_array,
                    point.pos,
                    marker_size/2,
                    marker_size*0.1,
                    marker_color,
                    12);
                break;
            case MarkerType::SQUARE:
                addSquare(
                    vertex_array,
                    point.pos,
                    marker_size,
                    marker_color);
                break;
        }
    }

    dirty = false;
}


void PointCloud::draw(sf::RenderTarget& target, sf::RenderStates states)const
{
    updateVertices();
    sf::Transform transform;
    transform.translate(pose.position().x(), pose.position().y());
    transform.rotate(pose.orientation());
    states.transform *= transform;
    target.draw(vertex_array, states);
}
