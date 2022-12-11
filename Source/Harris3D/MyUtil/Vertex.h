#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"
#include "VertexType.h"
#include "MeshIO.h"

struct Projection {    
    Eigen::Vector3d p;
    double d;
    int fIdx;
};

class Vertex {
public:
    // outgoing halfedge
    HalfEdgeIter he;
    
    // location in 3d
    Eigen::Vector3d position;
    
    // uv coords
    Eigen::Vector2d uv;
    
    // feature
    Eigen::VectorXd descriptor;
    
    // projection
    Projection projection;
  
    // 2pi - ∑ø
    double angleDefect() const;
 
    // id between 0 and |V|-1
    int index;
           
    // checks if vertex is contained in any edge or face
    bool isIsolated() const;
    
    // checks if this vertex lies on boundary
    bool isBoundary() const;
    
    // returns degree
    int degree() const;
    
    // returns angle weighted vertex normal
    Eigen::Vector3d normal() const;
    
    // returns area of barycentric dual cell associated with the vertex
    double dualArea() const;
    
    // checks if vertex is a feature
    bool isFeature(int t, int depth = 2) const;

	// 버텍스 타입
	EVertexType	getVertexType(const MeshData& meshData, const float dotFlat0, const float dotFlat1, int depth = 2) const;
	
	// 버텍스 노멀 타입
	EVertexNormalType getVertexNormalType (const MeshData& meshData, const float dotUp, const float dotDown) const;
    // ============ mesh saliency =======================

    // mean curvature
    double meanCurvature;
    
    // saliency
    double saliency;

    // vertex distances
    std::unordered_map<int, double> sqDistances;

    // computes saliency given cut-off distance
    double computeWeightedCurvature(VertexIter root, const double distance2);
    
    // checks is saliency is a maximum in local neighborhood
    bool isPeakSaliency(VertexIter root,
                        const std::vector<double>& levelSaliencies = std::vector<double>()) const;
};

#endif
