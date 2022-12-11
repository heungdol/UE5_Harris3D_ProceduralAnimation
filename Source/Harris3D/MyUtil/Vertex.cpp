#include "Vertex.h"

#include <ThirdParty/hlslcc/hlslcc/src/hlslcc_lib/compiler.h>

#include "HalfEdge.h"
#include "Face.h"

std::vector<HalfEdge> isolated;

bool Vertex::isIsolated() const
{
    return he == isolated.begin();
}

bool Vertex::isBoundary() const
{
    HalfEdgeCIter h = he;
    do {
        if (h->onBoundary) return true;
        
        h = h->flip->next;
    } while (h != he);
    
    return false;
}

int Vertex::degree() const
{
    int k = 0;
    HalfEdgeCIter h = he;
    do {
        k++;
        
        h = h->flip->next;
    } while (h != he);
    
    return k;
}

Eigen::Vector3d Vertex::normal() const
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    if (isIsolated()) return normal;
    
    HalfEdgeCIter h = he;
    do {
        Eigen::Vector3d e1 = h->next->vertex->position - position;
        Eigen::Vector3d e2 = h->next->next->vertex->position - position;
        
        double d = e1.dot(e2) / sqrt(e1.squaredNorm() * e2.squaredNorm());
        if (d < -1.0) d = -1.0;
        else if (d >  1.0) d = 1.0;
        double angle = acos(d);
        
        Eigen::Vector3d n = h->face->normal();
        normal += angle * n;
        
        h = h->flip->next;
    } while (h != he);
    
    if (!normal.isZero()) normal.normalize();
    return normal;
}

double Vertex::dualArea() const
{
    double area = 0.0;
    
    HalfEdgeCIter h = he;
    do {
        area += h->face->area();
        h = h->flip->next;
        
    } while (h != he);
    
    return area / 3.0;
}

double Vertex::angleDefect() const
{
    double defect = 2 * M_PI;
    
    HalfEdgeCIter h = he;
    do {
        Eigen::Vector3d u = h->next->vertex->position - h->vertex->position;
        Eigen::Vector3d v = h->next->next->vertex->position - h->vertex->position;
        
        double theta = acos(u.dot(v) / (u.norm() * v.norm()));
        defect -= theta;
        
        h = h->flip->next;
        
    } while (h != he);
    
    return defect;
}

bool Vertex::isFeature(int t, int depth) const
{
    std::queue<const Vertex *> queue;
    std::unordered_map<int, bool> visited;
    
    // enqueue
    queue.push(this);
    queue.push(NULL);
    visited[index] = true;
    int levels = 0;
    
    // perform bfs
    while (!queue.empty()) {
        const Vertex *v = queue.front();
        queue.pop();
        
        if (v == NULL) {
            levels++;
            queue.push(NULL);
            if (queue.front() == NULL || levels == depth) break;
            
        } else {
            HalfEdgeCIter h = v->he;
            do {
                const Vertex *vn = &(*h->flip->vertex);
                if (!visited[vn->index]) {
                    // double des = descriptor(t);
                    // double _des = vn->descriptor(t);
                    // check if descriptor value for a particular t is greater than the neighbor's value
                    if (descriptor(t) < vn->descriptor(t)) return false;
                    
                    queue.push(vn);
                    visited[vn->index] = true;
                }
                
                h = h->flip->next;
            } while (h != v->he);
        }
    }
    
    return true;
}

EVertexType Vertex::getVertexType(const MeshData& meshData, const float dotFlat0, const float dotFlat1, int depth) const
{
    EVertexType ret = EVertexType::NONE;
    
    std::queue<const Vertex *> queue;
    std::unordered_map<int, bool> visited;
    
    // enqueue
    queue.push(this);
    queue.push(NULL);
    visited[index] = true;
    int levels = 0;

    std::vector<int> neighborIndices = std::vector<int>();
    
    // 주변 이웃 인덱스 구하기
    while (!queue.empty()) {
        const Vertex *v = queue.front();
        queue.pop();
        
        if (v == NULL) {
            levels++;
            queue.push(NULL);
            if (queue.front() == NULL || levels == depth) break;
            
        } else {
            HalfEdgeCIter h = v->he;
            do {
                const Vertex *vn = &(*h->flip->vertex);
                
                if (!visited[vn->index])
                {
                    neighborIndices.push_back(vn->index);
                }
                
                h = h->flip->next;
            } while (h != v->he);
        }
    }

    // 얻은 주변 인덱스를 이용하여 Type 계산
    if (neighborIndices.size() == 0)
        return EVertexType::NONE;

    FVector neighboursPos_sum = FVector::Zero();
    FVector neighboursPos_avg = FVector::Zero();

    for (int i = 0; i != neighborIndices.size(); i++)
        neighboursPos_sum += FVector(meshData.positions[neighborIndices[i]].x(), meshData.positions[neighborIndices[i]].y(), meshData.positions[neighborIndices[i]].z());

    neighboursPos_avg = neighboursPos_sum / (1.0 * neighborIndices.size());

    FVector direction_self = FVector(meshData.normals[index].x(), meshData.normals[index].y(), meshData.normals[index].z());
    FVector direction_avg = FVector(meshData.positions[index].x(), meshData.positions[index].y(), meshData.positions[index].z()) - neighboursPos_avg;
    direction_avg = direction_avg / FVector::Distance(FVector::Zero(), direction_avg);

    float dotP = FVector::DotProduct(direction_self, direction_avg);
    
    if (dotFlat0 < dotP && dotP <= dotFlat1)
    {
        ret = EVertexType::VERTEX_FLAT;
    }
    else
    {
        if (dotP > 0)
        {
            ret = EVertexType::VERTEX_BUMP;
        }
        else
        {
            ret = EVertexType::VERTEX_SINK;
        }
    }
    
    return ret;
}

EVertexNormalType Vertex::getVertexNormalType(const MeshData& meshData, const float dotUp, const float dotDown) const
{
    FVector nor = FVector(meshData.normals[index].x(), meshData.normals[index].y(), meshData.normals[index].z());
    
    // 노멀 구분하기  
    float dot = FVector::DotProduct(nor, FVector::UpVector);

    if (dot > dotUp)
    {
        return EVertexNormalType::VERTEX_UP;
    }

    if (dot < dotDown)
    {
        return EVertexNormalType::VERTEX_DOWN;
    }

    return EVertexNormalType::VERTEX_PARALLEL;
}

double Vertex::computeWeightedCurvature(VertexIter root, const double distance2)
{
    std::queue<VertexIter> queue;
    queue.push(root);
    
    std::unordered_map<int, bool> visitedNeighbors;
    visitedNeighbors[index] = true;
    
    // initialize weighted gaussian curvatures
    double weightedCurvature = 0.0;
    double sumExponents = 0.0;
    
    // traverse n ring neighbors
    while (!queue.empty()) {
        VertexIter v = queue.front();
        queue.pop();
        
        HalfEdgeIter h = v->he;
        do {
            int vIndex = h->flip->vertex->index;
            if (!visitedNeighbors[vIndex]) {
                
                VertexIter nv = h->flip->vertex;
                
                if (sqDistances[vIndex] == 0.0) { // cache vertex distances
                    sqDistances[vIndex] = (nv->position - position).squaredNorm();
                    nv->sqDistances[index] = sqDistances[vIndex];
                } 
                
                if (sqDistances[vIndex] < 4 * distance2) {
                    double exponent = exp(-sqDistances[vIndex] / (2 * distance2));
                    weightedCurvature += nv->meanCurvature * exponent;
                    sumExponents += exponent;
                    queue.push(nv);
                }
                
                visitedNeighbors[vIndex] = true;
            }
            
            h = h->flip->next;
            
        } while (h != v->he);
    }

    if (sumExponents > 0.0) return weightedCurvature / sumExponents;
    return 0.0;
}


bool Vertex::isPeakSaliency(VertexIter root, const std::vector<double>& levelSaliencies) const
{
    std::queue<VertexIter> queue;
    queue.push(root);
    
    std::unordered_map<int, bool> visitedNeighbors;
    visitedNeighbors[index] = true;
    
    // traverse 2 ring neighbors
    bool traversed1Ring = false;
    while (!queue.empty()) {
        VertexIter v = queue.front();
        queue.pop();
    
        HalfEdgeIter h = v->he;
        do {
            int vIndex = h->flip->vertex->index;
            if (!visitedNeighbors[vIndex]) {
                
                if (levelSaliencies.empty()) {
                    if (saliency < h->flip->vertex->saliency) return false;
            
                } else {
                    if (levelSaliencies[index] < levelSaliencies[h->flip->vertex->index]) return false;
                }
    
                if (!traversed1Ring) queue.push(h->flip->vertex);
                visitedNeighbors[vIndex] = true;
            }
            
            h = h->flip->next;
        
        } while (h != v->he);
        
        traversed1Ring = true;
    }
    
    return true;
}
