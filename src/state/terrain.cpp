
#include "state/terrain.h"
#include <stack>
#include "utils/render_utils.h"


// ====== Terrain =====

void Terrain::addElementLandmarks(const Element& element)
{
    for (const auto& vertex: element.vertices) {
        landmarks.points.push_back(Point(element.pos + vertex));
        landmarks.descriptors.push_back(randomLandmarkDescriptor());
    }
}

Eigen::VectorXd Terrain::randomLandmarkDescriptor()const
{
    const size_t N = 8;

    Eigen::VectorXd v;
    v.resize(N);
    for (size_t i = 0; i < N; i++) {
        v(i) = (double)rand() / RAND_MAX;
    }
    v /= v.sum();
    return v;
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


void Terrain::getObservableLandmarks(const Pose& pose, PointCloud& landmarks_out)const
{
    landmarks_out.true_pose = pose;
    landmarks_out.points.clear();
    landmarks_out.descriptors.clear();

    static constexpr double intersection_allowance = 0.1;
    for (size_t i = 0; i < landmarks.points.size(); i++) {

        const Point& point = landmarks.points[i];
        const Eigen::VectorXd& descriptor = landmarks.descriptors[i];

        Eigen::Vector2d disp = point.pos - pose.position();
        double dist = disp.norm();
        double angle = std::atan2(disp.y(), disp.x()) - pose.orientation();
        double intersect_dist = queryIntersection(pose, angle);

        if (dist < intersect_dist + intersection_allowance) {
            landmarks_out.points.push_back(Point(dist, angle));
            landmarks_out.descriptors.push_back(descriptor);
        }
    }
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
        terrain.addElementLandmarks(element);
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
        terrain.addElementLandmarks(element);
    }
    { // Right
        Terrain::Element element;
        element.addVertex(x1, -y1);
        element.addVertex(x1-0.08, -0.5*y1);
        element.addVertex(x1-0.05, 0.1*y1);
        element.addVertex(x1-0.2, 0.7*y1);
        element.addVertex(x1, y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element);
    }
    { // Top
        Terrain::Element element;
        element.addVertex(x1, y1);
        element.addVertex(0.8*x1, y1-0.1);
        element.addVertex(0.1*x1, y1-0.2);
        element.addVertex(-0.4*x1, y1-0.1);
        element.addVertex(-x1, y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element);
    }
    { // Bottom
        Terrain::Element element;
        element.addVertex(-x1, -y1);
        element.addVertex(-0.4*x1, -y1+0.1);
        element.addVertex(0*x1, -y1+0.2);
        element.addVertex(0.5*x1, -y1+0.1);
        element.addVertex(x1, -y1);
        terrain.elements.push_back(element);
        terrain.addElementLandmarks(element);
    }
}
