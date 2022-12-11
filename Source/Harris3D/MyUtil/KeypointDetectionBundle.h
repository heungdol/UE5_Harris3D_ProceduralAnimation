#pragma once

#include "MyUtil.h"
#include "VertexType.h"
#include "KDTreeFlann.h"

#include "../Harris3D/MyMesh.h"
#include "../MyUtil/Mesh.h"

class KeypointDetectionBundle
{
public:
	Mesh mesh;
	MeshData* meshData;
	MyMesh myMesh;
	UStaticMeshComponent* m_pMeshCom;
	KDTreeFlann kdtree;

	// =====================================

	int m_ringSize = 5;
	double m_fraction = 0.01;
	double m_k = 0.04;
	//int m_vertexType_depth = 5;

	vector<double> harrisRPoints;

	// =====================================

	float m_t;
	int m_depth;

	Eigen::VectorXd evals;
	Eigen::MatrixXd evecs;

	// =====================================

	double m_saliencyRaidus = 1;
	double m_maxRadius = 10;
	double m_gamma_21 = 0.975f;
	double m_gamma_32 = 0.975f;
	int m_minNeighbors = 5;

	std::vector<double> eigenValues = std::vector<double>();

	// =====================================
	
	double m_cutoffSaliency = 0.75;

	// =====================================
	
	KeypointDetectionBundle () {};

	void SetParameters_Harris (int ringSize, double fraction, double k);
	void SetParameters_HKS (int t, int depth);
	void SetParameters_ISS (double saliencyRadius, double maxRadius, double gamma21, double gamma32, int minNeighbors);
	void SetParameters_MeshSaliency (double cutoff);
	
	bool InitMesh (UStaticMeshComponent*, MeshData*);

	void InitKeypoints_Harris (std::vector<int>&);//, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);
	void InitKeypoints_HKS (std::vector<int>&);//, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);
	void InitKeypoints_ISS (std::vector<int>&);//, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);
	void InitKeypoints_MeshSaliency (std::vector<int>&);//, TArray<int>&, TArray<FVector>&, TArray<FVector>&, TArray<EVertexType>&, TArray<EVertexNormalType>&);

	bool GetIsLocalMaxima(unsigned int);

	// compute eigenvalues and eigenvectors
	void computeEig(int K);//, const std::string& eigFilename);
    
	// compute hks
	void computeHks();
	
	// compute curvature descriptor
	void computeCurve();
    
	// normalize
	void normalize();

	// void buildAdjacency(Mesh *mesh, Eigen::SparseMatrix<double>& W);
	//
	// double computeMeanCurvature(Mesh *mesh, Eigen::VectorXd& H);
	//
	// void buildAreaMatrix(Mesh *mesh, Eigen::SparseMatrix<double>& A, const double scale);
	//
	// void extrapolateEvals(double& xhat, double& yhat, double& m, const Eigen::VectorXd& evals);
	//
	// void computeBinomialEntries(std::vector<Eigen::SparseMatrix<double>>& binomialSeries,
	// 						const Eigen::SparseMatrix<double>& L);
	//
	// void computeExponentialRepresentation(Eigen::SparseMatrix<double>& Kt, const double t,
	// 								  const std::vector<Eigen::SparseMatrix<double>>& binomialSeries);
	//
	// void sparsify(Eigen::SparseMatrix<double>& Kt, double eps);
	//
	// double computeGaussCurvature(Mesh *mesh, Eigen::VectorXd& K);
	//
	// void computeCurvatures(Mesh *mesh, Eigen::VectorXd& K, Eigen::VectorXd& H);
	//
	// void buildSimpleAverager(Mesh *mesh, Eigen::SparseMatrix<double>& L);

	std::vector<int> ComputeISSKeypoints(const std::vector<Eigen::Vector3d> &input,
											double salient_radius = 0.0,
											double non_max_radius = 0.0,
											double gamma_21 = 0.975,
											double gamma_32 = 0.975,
											int min_neighbors = 5);

	// computes mesh saliency
	void computeSaliency();
	
	void buildLaplacian(Eigen::SparseMatrix<double>& L) const;
    
	// computes mean curvature per vertex
	void computeMeanCurvature();

	//void CalculateMeshSaliency ();

};
