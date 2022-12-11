#include "PatchMatch.h"
#include "Mesh.h"

PatchMatch::PatchMatch(Mesh *mesh10, Mesh *mesh20):
mesh1(mesh10),
mesh2(mesh20),
distribution1(0, (int)mesh1->vertices.size()-1),
distribution2(0, (int)mesh2->vertices.size()-1)
{
    // TODO: compare regularly sampled geodesic fans 
    for (VertexCIter v1 = mesh1->vertices.begin(); v1 != mesh1->vertices.end(); v1++) {
        correspondenceMap[v1->index] = distribution2(generator);
    }
}

void PatchMatch::updateCorrespondence(VertexCIter v1, VertexCIter v2cand)
{
    VertexCIter v2curr = mesh2->vertices.begin() + correspondenceMap[v1->index];
    if ((v1->descriptor - v2curr->descriptor).squaredNorm() >
        (v1->descriptor - v2cand->descriptor).squaredNorm()) {
        correspondenceMap[v1->index] = v2cand->index;
    }
}

VertexCIter PatchMatch::traverse(VertexCIter v1)
{
    std::queue<VertexCIter> queue;
    std::unordered_map<int, bool> visited;
    
    // enqueue
    queue.push(v1);
    visited[v1->index] = true;

    // perform bfs
    while (!queue.empty()) {
        v1 = queue.front();
        queue.pop();
        
        HalfEdgeCIter h = v1->he;
        do {
            VertexCIter v1neigh = h->flip->vertex;
            if (!visited[v1neigh->index]) {
                queue.push(v1neigh);
                visited[v1neigh->index] = true;
                
            } else {
                VertexCIter v2neigh = mesh2->vertices.begin() + correspondenceMap[v1neigh->index];
                updateCorrespondence(v1, v2neigh);
            }
            
            h = h->flip->next;
        } while (h != v1->he);
    }
    
    return v1;
}

void PatchMatch::propogate()
{
    VertexCIter v1 = traverse(mesh1->vertices.begin() + distribution1(generator));
    traverse(v1);
}

void PatchMatch::performRandomSearch()
{
    for (VertexCIter v1 = mesh1->vertices.begin(); v1 != mesh1->vertices.end(); v1++) {
        for (int i = 0; i < 10; i++) { // TODO: sample points within decreasing radii
            VertexCIter v2rand = mesh2->vertices.begin() + distribution2(generator);
            updateCorrespondence(v1, v2rand);
        }
    }
}

void PatchMatch::compute(int iter)
{
    for (int i = 0; i < iter; i++) {
        propogate();
        performRandomSearch();
    }
}
