#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <fstream>

#include "KismetProceduralMeshLibrary.h"
#include "MeshDescription.h"
#include "MeshDescriptionBuilder.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshResources.h"

class face
{
private:
	
public:
	face() {}
	~face() {}
	
	int fIndex;
	std::vector<int> verts;
	
	std::vector<int> getvertices()
	{
		return verts;
	}

	int getv1()
	{
		return verts[0];
	}
	
	int getv2()
	{
		return verts[1];
	}
	
	int getv3()
	{
		return verts[2];
	}
	
	void addvertex(int vval)
	{
		verts.push_back(vval);
	}
	
	void addvertices(int v1, int v2, int v3)
	{
		verts.push_back(v1);
		verts.push_back(v2);
		verts.push_back(v3);
	}
};
