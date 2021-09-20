
#include "render_objects.h"

#include "render_utils.h"


// sf-like objects

namespace sf {

// ===== Arrow class =====

Arrow::Arrow():
    length(0.25),
    line_width(0.05),
    head_width(0.1),
    head_length(0.05),
    color(sf::Color::Black),
    dirty(true)
{
    vertex_array.setPrimitiveType(sf::Triangles);
}

void Arrow::setLength(double length)
{
    this->length = length;
    dirty = true;
}

void Arrow::setLineWidth(double line_width)
{
    this->line_width = line_width;
    dirty = true;
}

void Arrow::setHeadWidth(double head_width)
{
    this->head_width = head_width;
    dirty = true;
}

void Arrow::setHeadLength(double head_length)
{
    this->head_length = head_length;
    dirty = true;
}

void Arrow::setFillColor(sf::Color color)
{
    this->color = color;
    dirty = true;
}

void Arrow::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    addArrow(
        vertex_array,
        Pose(),
        length,
        line_width,
        head_width,
        head_length,
        color);

    dirty = false;
}

void Arrow::draw(sf::RenderTarget& target, sf::RenderStates states)const
{
    updateVertices();
    states.transform *= getTransform();
    target.draw(vertex_array, states);
}


// ===== RotArrow class =====

RotArrow::RotArrow():
    angle(M_PI/2),
    distance(0.25),
    line_width(0.05),
    head_width(0.1),
    head_length(0.05),
    color(sf::Color::Black),
    dirty(true)
{
    vertex_array.setPrimitiveType(sf::Triangles);
}

void RotArrow::setAngle(double angle)
{
    this->angle = angle;
    dirty = true;
}

void RotArrow::setDistance(double distance)
{
    this->distance = distance;
    dirty = true;
}

void RotArrow::setLineWidth(double line_width)
{
    this->line_width = line_width;
    dirty = true;
}

void RotArrow::setHeadWidth(double head_width)
{
    this->head_width = head_width;
    dirty = true;
}

void RotArrow::setHeadLength(double head_length)
{
    this->head_length = head_length;
    dirty = true;
}

void RotArrow::setFillColor(sf::Color color)
{
    this->color = color;
    dirty = true;
}

void RotArrow::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    addRotArrow(
        vertex_array,
        Pose(),
        angle,
        distance,
        line_width,
        head_width,
        head_length,
        color);

    dirty = false;
}

void RotArrow::draw(sf::RenderTarget& target, sf::RenderStates states)const
{
    updateVertices();
    states.transform *= getTransform();
    target.draw(vertex_array, states);
}


// ===== PoseMarker =====

PoseMarker::PoseMarker()
{
    setRadius(0.1);
    setColor(sf::Color::Black);
}

void PoseMarker::setRadius(double radius)
{
    circle.setOrigin(radius, radius);
    circle.setRadius(radius);

    arrow.setOrigin(0, 0);
    arrow.setLength(3*radius);
    arrow.setLineWidth(0.5*radius);
    arrow.setHeadWidth(radius);
    arrow.setHeadLength(radius);
}

void PoseMarker::setColor(sf::Color color)
{
    circle.setFillColor(color);
    arrow.setFillColor(color);
}

void PoseMarker::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getTransform();
    target.draw(circle, states);
    target.draw(arrow, states);
}


// ===== VelocityMarker =====

VelocityMarker::VelocityMarker()
{
    setLineWidth(0.05);
    setColor(sf::Color::Black);
    setLinearScale(0.5);
    setAngularScale(0.5);

    angular_arrow.setDistance(0.5);
}

void VelocityMarker::setLineWidth(double line_width)
{
    linear_arrow.setLineWidth(line_width);
    linear_arrow.setHeadWidth(line_width*3);
    linear_arrow.setHeadLength(line_width*3);
    angular_arrow.setLineWidth(line_width);
    angular_arrow.setHeadWidth(line_width*3);
    angular_arrow.setHeadLength(line_width*3);
}

void VelocityMarker::setLinearScale(double linear_scale)
{
    this->linear_scale = linear_scale;
    linear_arrow.setLength(velocity.linear().norm() * linear_scale);
}

void VelocityMarker::setAngularScale(double angular_scale)
{
    this->angular_scale = angular_scale;
    angular_arrow.setAngle(velocity.angular() * angular_scale);
}

void VelocityMarker::setColor(sf::Color color)
{
    linear_arrow.setFillColor(color);
    angular_arrow.setFillColor(color);
}

void VelocityMarker::setVelocity(const Velocity& velocity)
{
    this->velocity = velocity;
    linear_arrow.setLength(velocity.linear().norm() * linear_scale);
    linear_arrow.setRotation(std::atan2(velocity.linear().y(), velocity.linear().x()));
    angular_arrow.setAngle(velocity.angular() * angular_scale);
}

void VelocityMarker::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getTransform();
    target.draw(linear_arrow, states);
    target.draw(angular_arrow, states);
}


// ===== Marker =====

Marker::Marker():
    size(0.1),
    thickness(0.02),
    color(sf::Color::Red),
    dirty(true)
{
    vertex_array.setPrimitiveType(sf::Triangles);
}

void Marker::setSize(double size)
{
    this->size = size;
    dirty = true;
}

void Marker::setThickness(double thickness)
{
    this->thickness = thickness;
    dirty = true;
}

void Marker::setColor(sf::Color color)
{
    this->color = color;
    dirty = true;
}

void Marker::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    addMarker(
        vertex_array,
        Eigen::Vector2d(0,0),
        size,
        thickness,
        color);

    dirty = false;
}

void Marker::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    updateVertices();
    states.transform *= getTransform();
    target.draw(vertex_array, states);
}


// ===== MarkerArray =====

MarkerArray::MarkerArray()
{
    setSize(0.1);
    setThickness(0.02);
    setColor(sf::Color::Blue);
    vertex_array.setPrimitiveType(sf::Triangles);
}

void MarkerArray::setSize(double size)
{
    this->size = size;
    dirty = true;
}

void MarkerArray::setThickness(double thickness)
{
    this->thickness = thickness;
    dirty = true;
}

void MarkerArray::setColor(sf::Color color)
{
    this->color = color;
    dirty = true;
}

void MarkerArray::clearMarkers()
{
    markers.clear();
    dirty = true;
}

void MarkerArray::addMarker(const Eigen::Vector2d& position)
{
    markers.push_back(position);
    dirty = true;
}

void MarkerArray::updateVertices()const
{
    if (!dirty) return;

    vertex_array.clear();
    for (const auto& marker: markers) {
        ::addMarker(
            vertex_array,
            marker,
            size,
            thickness,
            color);
    }

    dirty = false;
}

void MarkerArray::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    updateVertices();
    states.transform *= getTransform();
    target.draw(vertex_array, states);
}

}
