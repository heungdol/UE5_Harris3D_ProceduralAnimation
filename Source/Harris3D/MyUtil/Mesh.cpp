#include "Mesh.h"
#include "MeshIO.h"

Mesh::Mesh()
{
}

Mesh::Mesh(const UStaticMeshComponent* sm)
{
    Clear ();
    
    isEnableModel = read (sm);
}

bool Mesh::read(const UStaticMeshComponent* sm)
{
    /*std::ifstream in(fileName.c_str());

    if (!in.is_open()) {
        std::cerr << "Error: Could not open file for reading" << std::endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        name = fileName;
        normalize();
    }
    
    in.close();*/

    //TODO HKS 막아놓음
    //return false;

    bool readSuccessful = MeshIO::read(sm, *this, meshData);
    if (readSuccessful)
    {
        normalize();
    }
        
    return readSuccessful;
}

/*bool Mesh::write(const std::string& fileName) const
{
    std::ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return false;
    }
    
    MeshIO::write(out, *this);
    
    out.close();
    return false;
}*/

bool Mesh::GetIsEnableModel()
{
    if (vertices.size() <= 0)
        return false;
        
    return isEnableModel;
}

FVector Mesh::GetVertexLocByIndex(int ii)
{
    if (verts.size() <= ii)
        return FVector (0, 0, 0);

    return verts[ii];
}

FVector Mesh::GetVertexNorByIndex(int ii)
{
    if (nors.size() <= ii)
        return FVector (0, 0, 0);

    return nors [ii];
}

void Mesh::normalize()
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin and determine radius
    double rMax = 0;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
