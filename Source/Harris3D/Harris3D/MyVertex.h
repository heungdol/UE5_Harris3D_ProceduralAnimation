#pragma once

#include <set>

#include "KismetProceduralMeshLibrary.h"
#include "MeshDescription.h"

#include "../MyUtil/VertexType.h"

#include "MyMesh.h"

using namespace std;

class MyMesh;
class MyFace;
class MyVertex
{
public:
	MyVertex ();
	~MyVertex ();

	double x;
	double y;
	double z;

	FVector vNormal;
	int vIndex;
	set<int> neighbours;

	EVertexType vType = EVertexType::NONE;

	double GetX();
	double GetY();
	double GetZ();
	
	void SetX(double xval);
	void SetY(double yval);
	void SetZ(double zval);
	void SetXYZ(double xval, double yval, double zval);

	FVector GetVertexNormal ();
	void SetVertexNormal (FVector vn);
	
	void AddNeighbour(int v);
	set<int> GetNeighbours();
	//TArray <int> GetNeighbours_TArray ();

	//void CalculateVertexType (MyMesh* myMesh, int ringSize, float dotFlat);
	
	EVertexType GetVertexType (MyMesh* myMesh, int ringSize, const float dotFlat0, const float dotFlat1);
	EVertexNormalType GetVertexNormalType (const float dotUp, const float dotDown);
};
