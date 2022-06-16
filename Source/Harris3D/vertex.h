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


class vertex
{
public:
	vertex() {}
	~vertex() {}
	
	double x;
	double y;
	double z;

	FVector vnormal;
	
	int vIndex;
	std::set<int> neighbours;

	double getx()
	{
		return x;
	}
	
	double gety()
	{
		return y;
	}
	
	double getz()
	{
		return z;
	}

	FVector GetVertexNormal ()
	{
		return vnormal;
	}
	
	void setx(double xval)
	{
		x = xval;
	}
	
	void sety(double yval)
	{
		y = yval;
	}
	
	void setz(double zval)
	{
		z = zval;
	}

	void SetVertexNormal (FVector vn)
	{
		vnormal = vn;
	}
	
	void setxyz(double xval, double yval, double zval)
	{
		x = xval;
		y = yval;
		z = zval;
	}
	
	void addNeighbour(int v)
	{
		neighbours.insert(v);
	}
	
	std::set<int> getNeighbours()
	{
		return neighbours;
	}

	
	TArray <int> GetNighbours ()
	{
		TArray <int> ret;

		std::set <int>::iterator it;

		for (it = neighbours.begin(); it != neighbours.end(); it++)
		{
			ret.Push(*it);
		}
		
		return ret;
	}
};