#ifndef PATCHMATCH_H
#define PATCHMATCH_H

#include "Types.h"
#include <random>

class PatchMatch {
public:
    // constructor
    PatchMatch(Mesh *mesh10, Mesh *mesh20);
    
    // compute
    void compute(int iter);
    
    // member variable
    std::unordered_map<int, int> correspondenceMap;
    
private:
    // update correspondece
    void updateCorrespondence(VertexCIter v1, VertexCIter v2cand);
    
    // traverse
    VertexCIter traverse(VertexCIter v1);
    
    // propogate
    void propogate();
    
    // perform random search
    void performRandomSearch();
    
    // member variables
    Mesh *mesh1;
    Mesh *mesh2;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution1, distribution2;
};

#endif
