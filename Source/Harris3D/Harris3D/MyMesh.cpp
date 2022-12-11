#include "MyMesh.h"
#include <algorithm>

MyMesh::MyMesh()
{
	
}

MyMesh::MyMesh (const UStaticMeshComponent* sm)
{
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("11"));
	
	/*vertices.clear();
	faces.clear();
	overlappingVert.clear ();*/
	
	if (ReadFile(sm) == false)
		return;
	
	double xmin=0.0, xmax=0.0, ymin=0.0, ymax=0.0, zmin=0.0, zmax=0.0;

	for(int k = 0; k < meshData.positions.size(); k++)
	{
		if(meshData.positions[k].x() < xmin)
			xmin = meshData.positions[k].x();
		else if(meshData.positions[k].x() > xmax)
			xmax = meshData.positions[k].x();

		if(meshData.positions[k].y() < ymin)
			ymin = meshData.positions[k].y();
		else if(meshData.positions[k].y() > ymax)
			ymax = meshData.positions[k].y();

		if(meshData.positions[k].z() < zmin)
			zmin = meshData.positions[k].z();
		else if(meshData.positions[k].z() > zmax)
			zmax = meshData.positions[k].z();
	}

	diagValue = sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));
}

MyMesh::~MyMesh()
{
	
}


bool MyMesh::ReadFile(const UStaticMeshComponent* sm)
{
	int numVertices, numFaces;;
	
	/*
	// Static Mesh 정보 가져오기
	TArray <FVector> verts;
	TArray <int> tris;
	TArray <FVector> nors;
	TArray <FVector2D> uvs;
	TArray<FProcMeshTangent> tans;

	UKismetProceduralMeshLibrary::GetSectionFromStaticMesh(sm->GetStaticMesh(), 0, 0, verts, tris, nors, uvs, tans);
	*/

	meshData.indices.clear();
	meshData.positions.clear();
	meshData.normals.clear();
	meshData.uvs.clear();

	if (!MyUtil::ReadMeshWithoutOverwrap (sm, meshData))
		return false;
	
	numVertices = meshData.positions.size();
	numFaces = meshData.indices.size();

	/*if (numVertices <= 0)
		return isEnableModel = false;

	// 버텍스 개수가 너무 많으면
	if (numVertices > VERTEX_NUMBER_MAX)
	{
		isEnableModel = false;
		return false;
	}*/

	isEnableModel = true;
	
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT(""+FString::FromInt(numVertices)));
	
 	// UStaticMesh->Vertex FVector
	//int count = 0;

	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(verts [0].X)));
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(verts [0].Y)));
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(verts [0].Z)));
	
	for(int i = 0; i < numVertices; i++)
	{
		double xc, yc, zc;

		xc = meshData.positions [i].x();
		yc = meshData.positions [i].y();
		zc = meshData.positions [i].z();

		MyVertex v;
		v.vIndex=i;
		v.SetXYZ(xc,yc,zc);

		xc = meshData.normals [i].x();
		yc = meshData.normals [i].y();
		zc = meshData.normals [i].z();
		v.SetVertexNormal(FVector (xc, yc, zc));

		//overlappingVert.push_back(i);
		//count++;
		
		// 중복 버텍스 제거 위함
		// 처음부터 순회하면서 같은 위치에 있는 것을 중복으로 판단한다
		// Mesh를 읽는 엔진 구조상 아래와 같은 번거로운 과정을 거친다

		/*for (int j = 0; j < overlappingVert.size()-1; j++)
		{
			// 거리로 판단
			float dist = FVector::Dist(verts[i], verts[j]);
			if (dist < 0.01f)
			{
				overlappingVert [i] = j;
				v.vIndex=j;
				//cout << "overlapping" << endl;
				//count--;
				break;
			}
		}*/
		
		vertices.push_back(v);
	}

	//G//Engine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(vertices[0].GetX())));
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(vertices[0].GetY())));
	///GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("OH: " + FString::FromInt(vertices[0].GetZ())));
	
	// UStaticMesh->Face Vertex 3개 묶음
	for(int i = 0;  i < numFaces; i++)
	{
		int vt1, vt2, vt3;
		
		vt1 = meshData.indices[i][0].position;
		vt2 = meshData.indices[i][1].position;
		vt3 = meshData.indices[i][2].position;

		MyFace f;
		f.fIndex=i;
		f.AddVertices(vt1,vt2,vt3);
		
		//create 1st ring neighbourhood, credit to source code
		vertices[vt1].AddNeighbour(vt2);
		vertices[vt1].AddNeighbour(vt3);
		
		vertices[vt2].AddNeighbour(vt1);
		vertices[vt2].AddNeighbour(vt3);
		
		vertices[vt3].AddNeighbour(vt1);
		vertices[vt3].AddNeighbour(vt2);

		faces.push_back(f);

	}

	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("true"));
	return true;
}

FVector MyMesh::GetVertexLocByIndex (int ii)
{
	if (vertices.size() <= ii)
		return FVector (0, 0, 0);

	return FVector ( vertices [ii].x,  vertices [ii].y,  vertices [ii].z);
}

FVector MyMesh::GetVertexNorByIndex (int ii)
{
	if (vertices.size() <= ii)
		return FVector (0, 0, 1);

	return vertices [ii].GetVertexNormal();
}

//calculate the neighbourhood at ring N
set<int> MyMesh::CalculateNeighbourhood_Ring(int indexVertex, int ringSize)
{
	set<int> s_prev, s_current, newring, s_total, s_ring, temp;
	set<int> nbhood = vertices[indexVertex].GetNeighbours();

	//set<int>::iterator iiii = nbhood.begin();
	//for (; indexVertex == 0 && iiii != nbhood.end (); iiii++)
	//	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT(""+ FString::FromInt(*iiii)));

	s_prev.insert(indexVertex); //insert the index of the vertex
	s_current.insert(nbhood.begin(), nbhood.end()); //insert the neighbourhood at ring 1
	s_total.insert(nbhood.begin(), nbhood.end()); //set of all neighbours of the vertex

	//store neighbours at each ring
	set<int>* nhd = new set<int>[ringSize];
	nhd[0].insert(nbhood.begin(), nbhood.end()); // at ring 1
	
	set<int> set_nhd;
	for (int j = 1; j < ringSize; ++j)
	{
		set<int>::iterator itr;
		for (itr = nhd[j - 1].begin(); itr != nhd[j - 1].end(); ++itr)
		{
			set_nhd = vertices[*itr].GetNeighbours();
			s_ring.insert(set_nhd.begin(), set_nhd.end()); //add neighbours of each vertex at ring j-1
			set_nhd.clear();
		}

		//calculating the difference s_ring - s_current
		set_difference(s_ring.begin(), s_ring.end(), s_current.begin(), s_current.end(), inserter(temp, temp.end()));

		//calculating the difference (s_ring - s_current) - s_prev
		set_difference(temp.begin(), temp.end(), s_prev.begin(), s_prev.end(), inserter(newring, newring.end()));

		s_prev.insert(s_current.begin(), s_current.end());
		s_current.insert(s_ring.begin(), s_ring.end());
		s_ring.clear();

		// add it to the array of rings
		s_total.insert(newring.begin(), newring.end());
		nhd[j].insert(newring.begin(), newring.end());
	}

	delete[] nhd;

	return s_total;
}

bool MyMesh::GetIsEnableModel()
{
	return isEnableModel;
}
