/*
#include "Bvh.h"
#include "Mesh.h"
#include <stack>

struct NodeEntry {
    // parent id
    int parentId;
    
    // range of objects covered by the node
    int startId, endId;
};

struct TraversalEntry {
    // constructor
    TraversalEntry(const int id0, const double d0) : id(id0), d(d0) {}
    
    // id
    int id;
    
    // distance
    double d;
};

Bvh::Bvh(Mesh *mesh0, const int& leafSize0):
mesh(mesh0),
faces(mesh->faces.size()),
nodeCount(0),
leafCount(0),
leafSize(leafSize0)
{
    
}

void Bvh::build()
{
    for (FaceCIter f = mesh->faces.begin(); f != mesh->faces.end(); f++) {
        faces[f->index] = *f;
    }
    
    int faceCount = (int)faces.size();
    
    NodeEntry nodeEntry;
    nodeEntry.parentId = -1;
    nodeEntry.startId = 0;
    nodeEntry.endId = faceCount;
    
    std::stack<NodeEntry> stack;
    stack.push(nodeEntry);
    flatTree.reserve(faceCount * 2);
    
    while (!stack.empty()) {
        // pop item off the stack and create a node
        nodeEntry = stack.top();
        stack.pop();
        int startId = nodeEntry.startId;
        int endId = nodeEntry.endId;
        
        nodeCount++;
        Node node;
        node.startId = startId;
        node.range = endId - startId;
        node.rightOffset = 2;
        
        // calculate bounding box
        BoundingBox bbox(faces[startId].boundingBox());
        BoundingBox centroid(faces[startId].centroid());
        for (int i = startId+1; i < endId; i++) {
            bbox.expandToInclude(faces[i].boundingBox());
            centroid.expandToInclude(faces[i].centroid());
        }
        node.boundingBox = bbox;
        
        // if node is a leaf
        if (node.range <= leafSize) {
            node.rightOffset = 0;
            leafCount++;
        }
        
        flatTree.push_back(node);
        
        // compute parent's rightOffset
        if (nodeEntry.parentId != -1) {
            flatTree[nodeEntry.parentId].rightOffset--;
            
            if (flatTree[nodeEntry.parentId].rightOffset == 0) {
                flatTree[nodeEntry.parentId].rightOffset = nodeCount - 1 - nodeEntry.parentId;
            }
        }
        
        // if a leaf, no need to subdivide
        if (node.rightOffset == 0) continue;
        
        // find the center of the longest dimension
        int maxDimension = centroid.maxDimension();
        double splitCoord = 0.5 * (centroid.min[maxDimension] + centroid.max[maxDimension]);
        
        // partition faces
        int mid = startId;
        for (int i = startId; i < endId; i++) {
            if (faces[i].centroid()[maxDimension] < splitCoord) {
                std::swap(faces[i], faces[mid]);
                mid++;
            }
        }
        
        // in case of a bad split
        if (mid == startId || mid == endId) {
            mid = startId + (endId - startId) / 2;
        }
        
        // push right child
        nodeEntry.startId = mid;
        nodeEntry.endId = endId;
        nodeEntry.parentId = nodeCount - 1;
        stack.push(nodeEntry);
        
        // push left child
        nodeEntry.startId = startId;
        nodeEntry.endId = mid;
        nodeEntry.parentId = nodeCount - 1;
        stack.push(nodeEntry);
    }
}

int Bvh::getIntersection(double& hit, Eigen::Vector3d& p, const Eigen::Vector3d& o) const
{
    int idx = 0;
    int index = -1;
    double dist1 = 0.0, dist2 = 0.0;
    
    TraversalEntry t(idx, -INFINITY);
    std::stack<TraversalEntry> stack;
    stack.push(t);
    
    while (!stack.empty()) {
        const TraversalEntry& t = stack.top();
        idx = t.id;
        stack.pop();
        
        const Node &node(flatTree[idx]);
        if (node.rightOffset == 0) { // node is a leaf
            for (int i = 0; i < node.range; i++) {
                const Face& f(faces[node.startId+i]);
                
                Eigen::Vector3d q;
                double dist = f.nearestPoint(q, o);
                if (dist < hit) {
                    index = f.index;
                    p = q;
                    hit = dist;
                } 
            }
            
        } else { // not a leaf
            bool hit0 = flatTree[idx+1].boundingBox.intersect(o, dist1);
            bool hit1 = flatTree[idx+node.rightOffset].boundingBox.intersect(o, dist2);
            
            // hit both bounding boxes
            if (hit0 && hit1) {
                int closer = idx + 1;
                int further = idx + node.rightOffset;
                
                if (dist2 < dist1) {
                    std::swap(dist1, dist2);
                    std::swap(closer, further);
                }
                
                // push farther node first
                stack.push(TraversalEntry(further, dist2));
                stack.push(TraversalEntry(closer, dist1));
                
            } else if (hit0) {
                stack.push(TraversalEntry(idx+1, dist1));
                
            } else if (hit1) {
                stack.push(TraversalEntry(idx+node.rightOffset, dist2));
            }
        }
    }
    
    return index;
}
*/
