#pragma once
#include <ThirdParty/Eigen/Eigen/Core>
#include "KismetProceduralMeshLibrary.h"
#include <limits>

#define DOTUPVECTOR_MAX 0.173648
#define DOTUPVECTOR_MIN -0.342020
#define VERTEXNUM_MAX 50000

class MyUtil;
class Index;
class MeshData;

class MyUtil
{
public:
	static bool ReadMeshWithoutOverwrap(const UStaticMeshComponent* sm, MeshData& meshData, float scale = 1);
	static bool IsValidVertexByNormal (Eigen::Vector3d);
	static bool IsValidVertexByNormal (FVector normal);
};

class Index {
public:
	Index() {}
    
	Index(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
	bool operator<(const Index& i) const {
		if (position < i.position) return true;
		if (position > i.position) return false;
		if (uv < i.uv) return true;
		if (uv > i.uv) return false;
		if (normal < i.normal) return true;
		if (normal > i.normal) return false;
        
		return false;
	}
    
	int position;
	int uv;
	int normal;
};

class MeshData {
public:
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Vector3d> uvs;
	std::vector<Eigen::Vector3d> normals;
	std::vector<std::vector<Index>> indices;   // 3개 단위
	std::vector <std::vector <int>> neighbors;
	FVector boundingBox_max;
	FVector boundingBox_min;

	void Clear ()
	{
		positions.clear();
		std::vector<Eigen::Vector3d>().swap(positions);

		uvs.clear();
		std::vector<Eigen::Vector3d>().swap(uvs);

		normals.clear();
		std::vector<Eigen::Vector3d>().swap(normals);

		normals.clear();
		std::vector<Eigen::Vector3d>().swap(normals);

		for (std::vector<Index> i : indices)
		{
			i.clear();
			std::vector<Index>().swap(i);
		}

		indices.clear();
		std::vector<std::vector<Index>>().swap(indices);

		for (std::vector <int> n : neighbors)
		{
			n.clear();
			std::vector <int>().swap(n);
		}
		
		neighbors.clear();
		std::vector<std::vector <int>>().swap(neighbors);

		boundingBox_max = FVector(-std::numeric_limits<double>::infinity());
		boundingBox_min = FVector(std::numeric_limits<double>::infinity());
	}

	double GetArea ()
	{
		double ret = 0;

		for (std::vector<Index> index : indices)
		{
			Eigen::Vector3d v0 = positions[index[0].position];
			Eigen::Vector3d v1 = positions[index[1].position];
			Eigen::Vector3d v2 = positions[index[2].position];

			Eigen::Vector3d a = v1 - v0;
			Eigen::Vector3d b = v2 - v0;

			Eigen::Vector3d cross = a.cross(b);

			ret += abs (cross.norm()) * 0.5;
		}

		ret *= 0.01 * 0.01;
		return ret;
	}

	double GetArea_Valid ()
	{
		double ret = 0;

		for (std::vector<Index> index : indices)
		{
			if (!MyUtil::IsValidVertexByNormal (normals [index[0].position])
				|| !MyUtil::IsValidVertexByNormal (normals [index[1].position])
				|| !MyUtil::IsValidVertexByNormal (normals [index[2].position]))
			{
				continue;
			}
			
			Eigen::Vector3d v0 = positions[index[0].position];
			Eigen::Vector3d v1 = positions[index[1].position];
			Eigen::Vector3d v2 = positions[index[2].position];

			Eigen::Vector3d a = v1 - v0;
			Eigen::Vector3d b = v2 - v0;

			Eigen::Vector3d cross = a.cross(b);

			ret += abs (cross.norm()) * 0.5;
		}

		ret *= 0.01 * 0.01;

		return ret;
	}

	int GetTotalVertexNumber ()
	{
		return positions.size();
	}

	int GetTotalVertexNumber_Valid ()
	{
		int ret = 0;

		for (int i = 0; i < normals.size(); i++)
		{
			if (MyUtil::IsValidVertexByNormal(normals[i]))
				ret++;
		}
		
		return ret;
	}

	FVector GetBoundingBoxSize ()
	{
		return boundingBox_max - boundingBox_min;
	}

	FVector GetBoundingBoxCoord_Min ()
	{
		return boundingBox_min;
	}

	FVector GetBoundingBoxCoord_Max ()
	{
		return boundingBox_max;
	}

	FVector GetVertexLocByIndex (int ii)
	{
		if (positions.size() <= ii)
			return FVector (0, 0, 0);

		return FVector (positions[ii].x(), positions[ii].y(), positions[ii].z());

	}
	
	FVector GetVertexNorByIndex (int ii)
	{
		if (normals.size() <= ii)
			return FVector (0, 0, 0);

		return FVector (normals [ii].x(), normals [ii].y(), normals [ii].z());
	}
};



