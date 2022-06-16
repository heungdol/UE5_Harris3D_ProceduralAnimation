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

#include "vertex.h"
#include "face.h"

class mesh
{
public:
	mesh() {}
	mesh(const UStaticMeshComponent* sm)
	{
		//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("11"));
		
		vertices.clear();
		faces.clear();
		overlappingVert.clear ();

		readOFF_File(sm);
		
		double xmin=0.0, xmax=0.0, ymin=0.0, ymax=0.0, zmin=0.0, zmax=0.0;

		for(int k = 0; k < vertices.size(); k++){

			if(vertices[k].getx() < xmin)
				xmin = vertices[k].getx();
			else if(vertices[k].getx() > xmax)
				xmax = vertices[k].getx();

			if(vertices[k].gety() < ymin)
				ymin = vertices[k].gety();
			else if(vertices[k].gety() > ymax)
				ymax = vertices[k].gety();

			if(vertices[k].getz() < zmin)
				zmin = vertices[k].getz();
			else if(vertices[k].getz() > zmax)
				zmax = vertices[k].getz();
		}

		diagValue = sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));

		//std::cout<<"Starting"<<std::endl;
	}

	// TODO 아래 함수는 필요없어 보이니 판단 후 주석처리
	/*mesh(const char* filename_vert, const char* filename_tri)
	{
		//load vertices and faces
		readFile_Intialize(filename_vert,filename_tri);

	}*/
	
	~mesh() {}

	std::vector<face> faces;
	std::vector<vertex> vertices;
	std::vector<int> overlappingVert;
	
	double diagValue; //the diagonal of the bounding box, used for clustering
	
	bool readOFF_File(const UStaticMeshComponent* sm)
	{
		int numVertices, numFaces;;
		
		// Static Mesh 정보 가져오기
		TArray <FVector> verts;
		TArray <int> tris;
		TArray <FVector> nors;
		TArray <FVector2D> uvs;
		TArray<FProcMeshTangent> tans;
		UKismetProceduralMeshLibrary::GetSectionFromStaticMesh(sm->GetStaticMesh(), 0, 0, verts, tris, nors, uvs, tans);
		
		numVertices = verts.Num();
		numFaces = tris.Num ();

		GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT(""+FString::FromInt(numVertices)));
		
 		// UStaticMesh->Vertex FVector
		int count = 0;
		for(int i = 0; i < numVertices; i++)
		{
			double xc, yc, zc;

			xc = verts [i].X;
			yc = verts [i].Y;
			zc = verts [i].Z;

			vertex v;
			v.vIndex=i;
			v.setxyz(xc,yc,zc);
			v.SetVertexNormal(nors [i]);

			overlappingVert.push_back(i);
			count++;
			
			// 중복 버텍스 제거 위함
			for (int j = 0; j < overlappingVert.size()-1; j++)
			{
				if (verts [i].X == verts [j].X && verts [i].Y == verts [j].Y && verts [i].Z == verts [j].Z)
				{
					overlappingVert [i] = j;
					v.vIndex=j;
					count--;
					break;
				}
			}
			
			vertices.push_back(v);
		}
		
		// UStaticMesh->Face Vertex 3개 묶음
		for(int i = 0, fi = 0;  i < numFaces; i+=3, fi++)
		{
			
			int vt1, vt2, vt3;
			
			vt1 = overlappingVert [tris[i+0]];
			vt2 = overlappingVert [tris[i+1]];
			vt3 = overlappingVert [tris[i+2]];

			face f;
			f.fIndex=fi;
			f.addvertices(vt1,vt2,vt3);
			
			//create 1st ring neighbourhood, credit to source code
			vertices[vt1].addNeighbour(vt2);
			vertices[vt1].addNeighbour(vt3);
			
			vertices[vt2].addNeighbour(vt1);
			vertices[vt2].addNeighbour(vt3);
			
			vertices[vt3].addNeighbour(vt1);
			vertices[vt3].addNeighbour(vt2);

			//if (vt1 == 0 || vt2 == 0 || vt3 == 0)
			//	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("true"));

			faces.push_back(f);

		}

		//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("true"));
		return true;
	}

	FVector GetVertexLocByIndex (int ii)
	{
		if (vertices.size() <= ii)
			return FVector (0, 0, 0);

		return FVector ( vertices [ii].x,  vertices [ii].y,  vertices [ii].z);
	}

	FVector GetVertexNorByIndex (int ii)
	{
		if (vertices.size() <= ii)
			return FVector (0, 0, 1);

		return vertices [ii].GetVertexNormal();
	}

};