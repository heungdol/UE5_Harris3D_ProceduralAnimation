/*
#ifndef BVH_H
#define BVH_H

#include "Types.h"
#include "BoundingBox.h"

class Node {
public:
    // member variables
    BoundingBox boundingBox;
    int startId, range, rightOffset;
};

class Bvh {
public:
    // constructor
    Bvh(Mesh *mesh0, const int& leafSize0 = 1);
    
    // builds the bvh
    void build();
    
    // returns face index
    int getIntersection(double& hit, Eigen::Vector3d& p, const Eigen::Vector3d& o) const;
    
private:
    // member variables
    Mesh *mesh;
    std::vector<Face> faces;
    int nodeCount, leafCount, leafSize;
    std::vector<Node> flatTree;
};

#endif
*/
