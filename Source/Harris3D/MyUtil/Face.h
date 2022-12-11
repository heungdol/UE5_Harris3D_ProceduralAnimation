#ifndef FACE_H
#define FACE_H

#include "Types.h"
#include "BoundingBox.h"

class Face {
public:
    ~Face ()
    {
        
    }
    
    // one of the halfedges associated with this face
    HalfEdgeIter he;
    
    // id between 0 and |F|-1
    int index;
    
    // quasi conformal error
    Eigen::Vector3d qcError;
    
    // checks if this face lies on boundary
    bool isBoundary() const;
    
    // returns normal to face
    Eigen::Vector3d normal() const;
    
    // returns face area
    double area() const;
    
    // returns centroid
    Eigen::Vector3d centroid() const;
    
    // returns bounding box
    BoundingBox boundingBox() const;
    
    // get nearest point on triangle
    double nearestPoint(Eigen::Vector3d& q, const Eigen::Vector3d& p) const;
};

#endif
