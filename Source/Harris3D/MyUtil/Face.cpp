#include "Face.h"
#include "HalfEdge.h"
#include "Vertex.h"

bool Face::isBoundary() const
{
    return he->onBoundary;
}

Eigen::Vector3d Face::normal() const
{
    const Eigen::Vector3d& a(he->vertex->position);
    const Eigen::Vector3d& b(he->next->vertex->position);
    const Eigen::Vector3d& c(he->next->next->vertex->position);

    //FVector A = FVector(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
    //FVector B = FVector(c[0]-a[0], c[1]-a[1], c[2]-a[2]);
    
    return (b-a).cross(c-a);
}

double Face::area() const
{
    if (isBoundary()) {
        return 0;
    }
    
    return 0.5 * normal().norm();
}

Eigen::Vector3d Face::centroid() const
{
    const Eigen::Vector3d& a(he->vertex->position);
    const Eigen::Vector3d& b(he->next->vertex->position);
    const Eigen::Vector3d& c(he->next->next->vertex->position);
    
    return (a + b + c) / 3.0;
}

BoundingBox Face::boundingBox() const
{
    if (isBoundary()) {
        return BoundingBox(he->vertex->position, he->next->vertex->position);
    }
    
    BoundingBox bbox(he->vertex->position);
    bbox.expandToInclude(he->next->vertex->position);
    bbox.expandToInclude(he->next->next->vertex->position);
    
    return bbox;
}

double Face::nearestPoint(Eigen::Vector3d& q, const Eigen::Vector3d& p) const
{
    const Eigen::Vector3d& a(he->vertex->position);
    const Eigen::Vector3d& b(he->next->vertex->position);
    const Eigen::Vector3d& c(he->next->next->vertex->position);
    
    Eigen::Vector3d e1 = b - a;
    Eigen::Vector3d e2 = c - a;
    Eigen::Vector3d e3 = c - b;
    
    // check if p is outside vertex region a
    Eigen::Vector3d v1 = p - a;
    double d1 = e1.dot(v1), d2 = e2.dot(v1);
    if (d1 <= 0 && d2 <= 0) {
        q = a;
        return (p-q).norm();
    }
    
    // check if p is outside vertex region b
    Eigen::Vector3d v2 = p - b;
    double d3 = e1.dot(v2), d4 = e2.dot(v2);
    if (d3 >= 0 && d4 <= d3) {
        q = b;
        return (p-q).norm();
    }
    
    // check if p is in edge region e1, if so return projection of p onto e1
    double vc = d1*d4 - d3*d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        double v = d1 / (d1-d3);
        q = a + v * e1;
        return (p-q).norm();
    }
    
    // check if p in vertex region outside c
    Eigen::Vector3d v3 = p - c;
    double d5 = e1.dot(v3), d6 = e2.dot(v3);
    if (d6 >= 0 && d5 <= d6) {
        q = c;
        return (p-q).norm();
    }
    
    // check if p is in edge region e2, if so return projection of p onto e2
    double vb = d5*d2 - d1*d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        double w = d2 / (d2-d6);
        q = a + w * e2;
        return (p-q).norm();
    }
    
    // check if p is in edge region e3, if so return projection of p onto e3
    double va = d3*d6 - d5*d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        q = b + w * e3;
        return (p-q).norm();
    }
    
    // p inside face region. Compute point through its barycentric coordinates (u,v,w)
    double d = 1.0 / (va + vb + vc);
    double v = vb * d;
    double w = vc * d;
    q = a + e1 * v + e2 * w;
    
    return (p-q).norm();
}
