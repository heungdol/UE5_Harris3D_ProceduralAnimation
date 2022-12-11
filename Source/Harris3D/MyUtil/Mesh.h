#ifndef MESH_H
#define MESH_H

#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "HalfEdge.h"
#include "MeshIO.h"
#include <ThirdParty/Eigen/Eigen/SparseCore>

class Mesh {
public:
    // default constructor
    Mesh ();
    Mesh(const UStaticMeshComponent* sm);

    // read mesh from file
    bool read(const UStaticMeshComponent* sm);
    
    // write mesh to file
    // bool write(const std::string& fileName) const;
    MeshData meshData = MeshData();

    // member variables
    std::vector<HalfEdge> halfEdges;
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<HalfEdgeIter> boundaries;
    std::string name;
    
    std::vector <FVector> verts;
    std::vector <FVector> nors;
    //int overlappingVertNum = 0;
    //std::vector<int> overlappingVert;

    bool isEnableModel;
    
    bool GetIsEnableModel ();
    FVector GetVertexLocByIndex (int ii);
    FVector GetVertexNorByIndex (int ii);

    void Clear ()
    {
        halfEdges.clear();
        std::vector<HalfEdge>().swap(halfEdges);
        
        vertices.clear();
        std::vector<Vertex>().swap(vertices);
        
        edges.clear();
        std::vector<Edge>().swap(edges);
        
        faces.clear();
        std::vector<Face>().swap(faces);
        
        boundaries.clear();
        std::vector<HalfEdgeIter>().swap(boundaries);

        verts.clear();
        std::vector<FVector>().swap(verts);
        
        nors.clear();
        std::vector<FVector>().swap(nors);

        meshData.Clear();
    }

private:
    // center mesh about origin and rescale to unit radius
    void normalize();
};

#endif
