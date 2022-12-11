#include "MyVertex.h"

MyVertex::MyVertex ()
{
	
}

MyVertex::~MyVertex ()
{
	
}

double MyVertex::GetX()
{
	return x;
}
	
double MyVertex::GetY()
{
	return y;
}
	
double MyVertex::GetZ()
{
	return z;
}

void MyVertex::SetX(double xval)
{
	x = xval;
}
	
void MyVertex::SetY(double yval)
{
	y = yval;
}
	
void MyVertex::SetZ(double zval)
{
	z = zval;
}

void MyVertex::SetXYZ(double xval, double yval, double zval)
{
	x = xval;
	y = yval;
	z = zval;
}



FVector MyVertex::GetVertexNormal ()
{
	return vNormal;
}


void MyVertex::SetVertexNormal (FVector vn)
{
	vNormal = vn;
}
	

	
void MyVertex::AddNeighbour(int v)
{
	neighbours.insert(v);
}
	
set<int> MyVertex::GetNeighbours()
{
	return neighbours;
}
	
/*TArray <int> MyVertex::GetNeighbours_TArray ()
{
	TArray <int> ret;

	set <int>::iterator it;

	for (it = neighbours.begin(); it != neighbours.end(); it++)
	{
		ret.Push(*it);
	}
		
	return ret;
}*/

EVertexType MyVertex::GetVertexType(MyMesh* myMesh, int ringSize, const float dotFlat0, const float dotFlat1)
{
	set <int> neighbours_ring = myMesh->CalculateNeighbourhood_Ring (vIndex, ringSize);
	
	if (neighbours_ring.size() == 0)
		return  EVertexType::NONE;

	FVector neighboursPos_sum = FVector::Zero();
	FVector neighboursPos_avg = FVector::Zero();

	set<int>::iterator iter = neighbours_ring.begin();
	for (; iter != neighbours_ring.end(); ++iter)
		neighboursPos_sum += myMesh->GetVertexLocByIndex(*iter);

	neighboursPos_avg = neighboursPos_sum / (1.0 * neighbours_ring.size());

	//cout << neighbours_ring.size() << endl;

	FVector direction_self = myMesh->GetVertexNorByIndex(vIndex);
	FVector direction_avg = myMesh->GetVertexLocByIndex(vIndex) - neighboursPos_avg;
	direction_avg = direction_avg / FVector::Distance(FVector::Zero(), direction_avg);
	
	float dotP = FVector::DotProduct(direction_self, direction_avg);
	
	if (dotFlat0 < dotP && dotP < dotFlat1)
	{
		return EVertexType::VERTEX_FLAT;
	}
	else
	{
		if (dotP > 0)
		{
			return EVertexType::VERTEX_BUMP;
		}
		else
		{
			return EVertexType::VERTEX_SINK;
		}
	}
	
	return  EVertexType::NONE;
}

EVertexNormalType MyVertex::GetVertexNormalType(const float dotUp, const float dotDown)
{
	// 노멀 구분하기  
	float dot = FVector::DotProduct(vNormal, FVector::UpVector);

	if (dot > dotUp)
	{
		return EVertexNormalType::VERTEX_UP;
	}
	
	if (dot < dotDown)
	{
		return EVertexNormalType::VERTEX_DOWN;
	}

	return EVertexNormalType::VERTEX_PARALLEL;
}
