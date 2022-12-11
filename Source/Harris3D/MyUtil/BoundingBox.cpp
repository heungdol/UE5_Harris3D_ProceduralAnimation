#include "BoundingBox.h"

BoundingBox::BoundingBox():
min(Eigen::Vector3d::Zero()),
max(Eigen::Vector3d::Zero()),
extent(Eigen::Vector3d::Zero())
{
    
}

BoundingBox::BoundingBox(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0):
min(min0),
max(max0)
{
    extent = max - min;
}

BoundingBox::BoundingBox(const Eigen::Vector3d& p):
min(p),
max(p)
{
    extent = max - min;
}

void BoundingBox::expandToInclude(const Eigen::Vector3d& p)
{
    if (min.isZero() && max.isZero()) {
        min = max = p;
        return;
    }
    
    if (min.x() > p.x()) min.x() = p.x();
    if (min.y() > p.y()) min.y() = p.y();
    if (min.z() > p.z()) min.z() = p.z();
    
    if (max.x() < p.x()) max.x() = p.x();
    if (max.y() < p.y()) max.y() = p.y();
    if (max.z() < p.z()) max.z() = p.z();
    
    extent = max - min;
}

void BoundingBox::expandToInclude(const BoundingBox& b)
{
    if (b.min.isZero() && b.max.isZero()) {
        min = b.min;
        max = b.max;
        return;
    }
    
    if (min.x() > b.min.x()) min.x() = b.min.x();
    if (min.y() > b.min.y()) min.y() = b.min.y();
    if (min.z() > b.min.z()) min.z() = b.min.z();
    
    if (max.x() < b.max.x()) max.x() = b.max.x();
    if (max.y() < b.max.y()) max.y() = b.max.y();
    if (max.z() < b.max.z()) max.z() = b.max.z();
    
    extent = max - min;
}

int BoundingBox::maxDimension() const
{
    int result = 0;
    if (extent.y() > extent.x()) result = 1;
    if (extent.z() > extent.y() && extent.z() > extent.x()) result = 2;
    
    return result;
}

bool BoundingBox::intersect(const Eigen::Vector3d& p, double& dist) const
{
    // hack
    if (min.x()-0.1 <= p.x() && p.x() <= max.x()+0.1 &&
        min.y()-0.1 <= p.y() && p.y() <= max.y()+0.1 &&
        min.z()-0.1 <= p.z() && p.z() <= max.z()+0.1) {
        
        dist = 0.0;
        return true;
    }
    
    dist = INFINITY;
    return false;
}
