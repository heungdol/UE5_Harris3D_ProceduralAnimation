#ifndef MESH_IO_H
#define MESH_IO_H

#include <fstream>
#include "Types.h"
#include "../MyUtil/MyUtil.h"

class MeshIO {
public:
    // reads data from obj file
    static bool read(const UStaticMeshComponent* sm, Mesh& mesh, MeshData& meshData);
    
    // reads eigenvectors and eigenvalues from file
    // static void readEig(std::ifstream& in, Eigen::VectorXd& evals, Eigen::MatrixXd& evecs);
    
    // writes data in obj format
    // static void write(std::ofstream& out, const Mesh& mesh);
    //
    // // writes eigenvectors and eigenvalues to file
    // static void writeEig(std::ofstream& out, Eigen::VectorXd& evals, Eigen::MatrixXd& evecs);
    //
    // // writes descriptor to file
    // static void writeDescriptor(std::ofstream& out, const Mesh& mesh);
    
    // builds the halfedge mesh
    static bool buildMesh(const MeshData& data, Mesh& mesh);
    
private:
    // reserves spave for mesh vertices, uvs, normals and faces
    static void preallocateMeshElements(const MeshData& data, Mesh& mesh);
    
    // sets index for elements
    static void indexElements(Mesh& mesh);
    
    // checks if any vertex is not contained in a face
    static void checkIsolatedVertices(const Mesh& mesh);
    
    // checks if a vertex is non-manifold
    static void checkNonManifoldVertices(const Mesh& mesh);
};

#endif
