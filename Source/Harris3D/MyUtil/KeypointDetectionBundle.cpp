#include "KeypointDetectionBundle.h"

#include "../spectra/include/MatOp/SparseSymMatProd.h"
#include "../spectra/include/MatOp/SparseCholesky.h"
#include "../spectra/include/SymGEigsSolver.h"
#include "../spectra/include/SymEigsShiftSolver.h"
#include "../spectra/include/SymEigsBase.h"

#include <ThirdParty/Eigen/Eigen/Eigenvalues>

#include <numeric>

#define N 10

void KeypointDetectionBundle::SetParameters_Harris(int ringSize, double fraction, double k)
{
	m_ringSize = ringSize;
	m_fraction = fraction;
	m_k = k;
}

void KeypointDetectionBundle::SetParameters_HKS(int t, int depth)
{
	m_t = t;
	m_depth = depth;
}

void KeypointDetectionBundle::SetParameters_ISS(double saliencyRadius, double maxRadius, double gamma21, double gamma32,
	int minNeighbors)
{
	m_saliencyRaidus = saliencyRadius;
	m_maxRadius = maxRadius;
	m_gamma_21 = gamma21;
	m_gamma_32 = gamma32;
	m_minNeighbors = minNeighbors;
}

void KeypointDetectionBundle::SetParameters_MeshSaliency(double cutoff)
{
	m_cutoffSaliency = cutoff;
}

bool KeypointDetectionBundle::InitMesh(UStaticMeshComponent* sm, MeshData* md)
{
	m_pMeshCom = sm;
	
	mesh.Clear();
	mesh = Mesh (sm);
	bool ret = mesh.GetIsEnableModel();

	meshData = md;
	meshData->Clear ();
	ret |= MyUtil::ReadMeshWithoutOverwrap(sm, *meshData);

	myMesh = MyMesh (sm);
	ret |= myMesh.GetIsEnableModel();

	return ret;
}

void KeypointDetectionBundle::InitKeypoints_Harris(std::vector<int>& vrts_selected)//, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
//	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	if (myMesh.GetIsEnableModel() == false)
		return;
	
	int vertexSize = myMesh.vertices.size();
	
	for (int indexVertex = 0; indexVertex < vertexSize; indexVertex++)
	{
		//vertexSize

		// 중복인 경우 계산하지 않고 컨티뉴
		/*if (indexVertex != myMesh.overlappingVert[indexVertex])
		{
			//harrisRPoints.push_back(harrisRPoints[myMesh.overlappingVert[indexVertex]]);
			harrisRPoints.push_back(0);
			continue;
		}*/

		vector<double> x_coord, y_coord, z_coord;
		//caculate the neighbourhood
		set<int> set_nhd;

		//calculate the k rings neighbourhood of each vertex
		set_nhd = myMesh.CalculateNeighbourhood_Ring(indexVertex, m_ringSize);

		set<int>::iterator itr;
		for (itr = set_nhd.begin(); itr != set_nhd.end(); ++itr)
		{
			//get the x,y,z coordinates
			x_coord.push_back(myMesh.vertices[*itr].GetX());
			y_coord.push_back(myMesh.vertices[*itr].GetY());
			z_coord.push_back(myMesh.vertices[*itr].GetZ());
		}

		//adding the vertex itself to the set, the last element
		x_coord.push_back(myMesh.vertices[indexVertex].GetX());
		y_coord.push_back(myMesh.vertices[indexVertex].GetY());
		z_coord.push_back(myMesh.vertices[indexVertex].GetZ());


		//calculate centroid of the neighbourhood Vk(v)
		int nhd_size = x_coord.size();

		double sum_x = std::accumulate(x_coord.begin(), x_coord.end(), 0.0);
		double averg_x = (double)sum_x / nhd_size;

		double sum_y = std::accumulate(y_coord.begin(), y_coord.end(), 0.0);
		double averg_y = (double)sum_y / nhd_size;

		double sum_z = std::accumulate(z_coord.begin(), z_coord.end(), 0.0);
		double averg_z = (double)sum_z / nhd_size;

		//apply PCA to get the normal of the fitting plane
		//using Eigen Library

		//translate the set of points so that centroid is on the origin
		//Matrix= 3*nhd_size

		Eigen::MatrixXd nhd_matrix(3, nhd_size);
		for (int jj = 0; jj < nhd_size; jj++)
		{
			//store them in Matrix
			//x_trans = x_coord - x_centroid
			nhd_matrix(0, jj) = x_coord[jj] - averg_x;
			nhd_matrix(1, jj) = y_coord[jj] - averg_y;
			nhd_matrix(2, jj) = z_coord[jj] - averg_z;
		}

		//Covariance matrix C
		// 1/n-1*X*Xt
		Eigen::Matrix3d CovM;
		CovM = (nhd_matrix * nhd_matrix.transpose()) / (nhd_size - 1); //creates a symmetric matrix

		// Calculate EigenVectors and EigenValues of Covaraince matrix
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(CovM);
		//SelfAdjointEigenSolver if the matrix is symmetric(faster)

		Eigen::MatrixXd eig_values(3, 1);
		eig_values = es.eigenvalues().real(); //sorted in increasing order
		Eigen::Matrix3d principal_comps = es.eigenvectors().real();

		//sort eigenvectors in decreasing order by swaping
		Eigen::MatrixXd tmp(3, 1);
		tmp = principal_comps.col(0);
		principal_comps.col(0) = principal_comps.col(2);
		principal_comps.col(2) = tmp;


		//set of points is rotated so that the normal of the fitting plane is the z-axis
		Eigen::MatrixXd rotated_points(3, nhd_size);
		rotated_points = principal_comps.transpose() * nhd_matrix;

		//translate the set of points so that the point v is in the origin of the XY-plane
		double x_vertex = rotated_points(0, nhd_size - 1);
		double y_vertex = rotated_points(1, nhd_size - 1);
		double z_vertex = rotated_points(2, nhd_size - 1);

		Eigen::MatrixXd trans_points(3, nhd_size);
		for (int jk = 0; jk < nhd_size; jk++)
		{
			//trans_points = rotated_points - vertex
			trans_points(0, jk) = rotated_points(0, jk) - x_vertex;
			trans_points(1, jk) = rotated_points(1, jk) - y_vertex;
			trans_points(2, jk) = rotated_points(2, jk) - z_vertex;
		}

		//fit a quadratic surface to the set of transformed points
		//z = f(x,y) =p1/2*x2 +p2*x*y + p3/2*y2 +p4*x +p5*y +p6
		Eigen::MatrixXd eqns(nhd_size, 6); // equations
		Eigen::MatrixXd bvector(nhd_size, 1);
		Eigen::MatrixXd xvector(6, 1);
		for (int kk = 0; kk < nhd_size; kk++)
		{
			double xv = trans_points(0, kk);
			double yv = trans_points(1, kk);
			double zv = trans_points(2, kk);

			bvector(kk, 0) = zv;

			eqns(kk, 0) = (xv * xv) / 2; //coefficient of p1
			eqns(kk, 1) = xv * yv; //p2
			eqns(kk, 2) = (yv * yv) / 2; //p3
			eqns(kk, 3) = xv; //p4
			eqns(kk, 4) = yv; //p5
			eqns(kk, 5) = 1; //p6
		}

		//solve the linear system Ax=b
		xvector = eqns.colPivHouseholderQr().solve(bvector);

		//extract the solution of the linear system
		double p1 = xvector(0, 0);
		double p2 = xvector(1, 0);
		double p3 = xvector(2, 0);
		double p4 = xvector(3, 0);
		double p5 = xvector(4, 0);
		double p6 = xvector(5, 0);

		double A = p4 * p4 + 2 * p1 * p1 + 2 * p2 * p2;
		double B = p4 * p4 + 2 * p2 * p2 + 2 * p3 * p3; //difference with source code p5 = p2 =0.3..
		double C = p4 * p5 + 2 * p1 * p2 + 2 * p2 * p3;

		//Harris operator value in the point v        
		double harrisV = (A * B) - (C * C) - m_k * ((A + B) * (A + B));
		harrisRPoints.push_back(harrisV);
	} //endforeachvertex

	//Pre-selection of the interest points
	//preserve the vertices which are local maximum
	// 주변 이웃한 버텍스 수가 가장 많은 버텍스
	vector<int> preselected;
	for (int nV = 0; nV < vertexSize; nV++)
	{
		/*// 중복 패스
		if (nV != myMesh.overlappingVert[nV])
		{
			continue;
		}*/
		
		bool localMaxima = GetIsLocalMaxima(nV);
		if (localMaxima == true)
		{
			preselected.push_back(nV);
		}
	}
	//sort the preselected vertices, decreasing order
	sort(preselected.rbegin(), preselected.rend());
	{
		//Selecting interest points
		vector<int> selectedVertices; //Highest Harris Method

		//Convert set to VectorXi
		int numPreselected = preselected.size();
		Eigen::VectorXi preSelectedVertexes(numPreselected);
		int ctrlVar1(0);
		for (vector<int>::iterator it = preselected.begin(); it != preselected.end(); ++it)
		{
			preSelectedVertexes(ctrlVar1) = *it;
			ctrlVar1++;
		}

		//Get vector with harris values
		Eigen::VectorXd preSelectedHarrisValues(numPreselected);
		for (int iPre = 0; iPre < numPreselected; iPre++)
		{
			preSelectedHarrisValues(iPre) = harrisRPoints[preSelectedVertexes(iPre)];
		}

		vector<int> _selectedVertices;

		double maxi(0);
		for (int iIP = 0; iIP < preSelectedVertexes.size(); iIP++)
		{
			maxi = preSelectedHarrisValues.maxCoeff();
			for (int i = 0; i < preSelectedVertexes.size(); i++)
			{
				if (abs(maxi - preSelectedHarrisValues(i)) < 0.00001)
				{
					_selectedVertices.push_back(preSelectedVertexes(i));
					preSelectedHarrisValues(i) = 0;
					break;
				}
			}
		}

		//sort the preselected vertices, decreasing order
		sort(preselected.rbegin(), preselected.rend());

		vrts_selected = _selectedVertices;
	}
	// for (int i = 0; i < vrts_selected.size(); i++)
	// 	vrts_postSelected.Add(vrts_selected[i]);

	// // 정렬 및 중복 제거
	// sort (vrts_selected.begin(), vrts_selected.end());
	// vrts_selected.erase (unique (vrts_selected.begin(), vrts_selected.end()), vrts_selected.end());
	//
	// // ...
	// for (int i = 0; i < vrts_selected.size(); i++)
	// 	vrts_postSelected.Add(vrts_selected[i]);
}

void KeypointDetectionBundle::InitKeypoints_HKS(std::vector<int>& vrts_selected)//, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
//	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	if (mesh.GetIsEnableModel() == false)
		return;
    
	computeEig(500);//, eigFilename);
	computeHks();
	normalize();

	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		if (v->isFeature(m_t, m_depth))
			vrts_selected.push_back(v->index);
	}

	// for (int vrts : vrts_selected)
	// 	vrts_postSelected.Add(vrts);
}

void KeypointDetectionBundle::InitKeypoints_ISS(std::vector<int>& vrts_selected)//, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
//	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	vrts_selected.clear();
	std::vector <int>().swap(vrts_selected);
	
	vrts_selected = ComputeISSKeypoints(meshData->positions, m_saliencyRaidus, m_maxRadius, m_gamma_21, m_gamma_32, m_minNeighbors);

	// for (int ind : vrts_selected)
	// {
	// 	vrts_postSelected.Add(ind);
	// }
}

void KeypointDetectionBundle::InitKeypoints_MeshSaliency(std::vector<int>& vrts_selected)//, TArray<int>& vrts_postSelected, TArray<FVector>& vrtLocs_postSelected
//	, TArray<FVector>& vrtNors_postSelected, TArray<EVertexType>& vrtTypes_postSelected, TArray<EVertexNormalType>& vrtNorTypes_postSelected)
{
	if (mesh.GetIsEnableModel() == false)
		return;

	computeSaliency ();

	for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e ++)
	{
		VertexIter a = e->he->vertex;
		VertexIter b = e->he->flip->vertex;

		if (a->saliency > m_cutoffSaliency && a->isPeakSaliency(a)) {
			// glVertex3d(a->position.x(), a->position.y(), a->position.z());
			vrts_selected.push_back(a->index);
		}
            
		if (b->saliency > m_cutoffSaliency && b->isPeakSaliency(b)) {
			// glVertex3d(b->position.x(), b->position.y(), b->position.z());
			vrts_selected.push_back(b->index);
		}
	}
	
	// for (int vrts : vrts_selected)
	// {
	// 	if (vrts_postSelected.Contains(vrts))
	// 		continue;
	// 	
	// 	vrts_postSelected.Add(vrts);
	// }
}

bool KeypointDetectionBundle::GetIsLocalMaxima(unsigned int vertexIndex)
{
	set<int> nhd = myMesh.CalculateNeighbourhood_Ring(vertexIndex, m_ringSize);
	//myMesh.vertices[vertexIndex].GetNeighbours();
	
	set<int>::iterator itrr;
	for (itrr = nhd.begin(); itrr != nhd.end(); ++itrr)
	{
		if (harrisRPoints[vertexIndex] < harrisRPoints[*itrr])
		{
			return false;
		}
	}
	
	return true;
}


void _buildAdjacency(Mesh *mesh, Eigen::SparseMatrix<double>& W)
{
    std::vector<Eigen::Triplet<double>> WTriplets;

    //int index = 0;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++)
        {
         //index++;
         //if ((index-1) != mesh->overlappingVert[index-1])
         //    continue;
        
        HalfEdgeCIter he = v->he;
        double sumCoefficients = 0.0;
        do {
            double coefficient = 0.5 * (he->cotan() + he->flip->cotan());
            if (coefficient < 0.0) coefficient = 0.0;
            sumCoefficients += coefficient;
            
            WTriplets.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, -coefficient));
            
            he = he->flip->next;
        } while (he != v->he);
        
        WTriplets.push_back(Eigen::Triplet<double>(v->index, v->index,
                                                   sumCoefficients + 1e-8));
    }
    
    W.setFromTriplets(WTriplets.begin(), WTriplets.end());
}

void _buildAreaMatrix(Mesh *mesh, Eigen::SparseMatrix<double>& A, const double scale)
{
    std::vector<Eigen::Triplet<double>> ATriplets;
    
    double sum = 0.0;
    //int index = 0;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++)
        {
        // index++;
        // if ((index-1) != mesh->overlappingVert[index-1])
        //     continue;
        
        double area = v->dualArea();
        ATriplets.push_back(Eigen::Triplet<double>(v->index, v->index, area));
        sum += area;
    }
    
    A.setFromTriplets(ATriplets.begin(), ATriplets.end());
    A *= scale/sum;
}

void _extrapolateEvals(double& xhat, double& yhat, double& m, const Eigen::VectorXd& evals)
{
    // compute averages
    const int K = (int)evals.size();
    xhat = 0.0; yhat = 0.0;
    for (int i = 0; i < K; i++) {
        xhat += i;
        yhat += evals(i);
    }
    xhat /= K; yhat /= K;
    
    // compute slope
    double den = 0.0; m = 0.0;
    for (int i = 0; i < K; i++) {
        m += (i - xhat)*(evals(i) - yhat);
        den += (i - xhat)*(i - xhat);
    }
    m /= den;
}

void _computeBinomialEntries(std::vector<Eigen::SparseMatrix<double>>& binomialSeries,
                            const Eigen::SparseMatrix<double>& L)
{
    int k = 0;
    Eigen::SparseMatrix<double> Id(L.cols(), L.cols()); Id.setIdentity();
    Eigen::SparseMatrix<double> Q = Id;
    for (int m = 0; m < (int)binomialSeries.size(); m++) {
        if (k == m-1) {
            Q = Q*(L - k*Id);
            k++;
        }
        
        if (m > 0) Q /= m;
        binomialSeries[m] = Q;
    }
}

void _computeExponentialRepresentation(Eigen::SparseMatrix<double>& Kt, const double t,
                                      const std::vector<Eigen::SparseMatrix<double>>& binomialSeries)
{
    Kt.setZero();
    for (int m = 0; m < (int)binomialSeries.size(); m++) {
        Kt += binomialSeries[m]*pow(exp(-t) - 1, m);
    }
}

void _sparsify(Eigen::SparseMatrix<double>& Kt, double eps)
{
    for (int i = 0; i < Kt.outerSize(); i++) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Kt, i); it; ++it) {
            if (it.valueRef() < eps) it.valueRef() = 0.0;
        }
    }
    Kt.prune(0.0);
}

double _computeGaussCurvature(Mesh *mesh, Eigen::VectorXd& K)
{
    double maxGauss = -INFINITY;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        K(v->index) = v->angleDefect() / v->dualArea();
        if (maxGauss < fabs(K(v->index))) maxGauss = fabs(K(v->index));
    }
    
    return maxGauss;
}

double _computeMeanCurvature(Mesh *mesh, Eigen::VectorXd& H)
{
    int vsize = (int)mesh->vertices.size();
    
    // build laplace matrix
    Eigen::SparseMatrix<double> L(vsize, vsize);
    _buildAdjacency(mesh, L);
    Eigen::SparseMatrix<double> A(vsize, vsize);
    _buildAreaMatrix(mesh, A, 1.0);
    L = A.cwiseInverse()*L;
    
    Eigen::MatrixXd x;
    x.resize((int)vsize, 3);
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        x.row(v->index) = v->position;
    }
    x = L*x;
    
    // set absolute mean curvature
    double maxMean = -INFINITY;
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        H(v->index) = 0.5 * x.row(v->index).norm();
        if (maxMean < H(v->index)) maxMean = H(v->index);
    }
    
    return maxMean;
}

void _computeCurvatures(Mesh *mesh, Eigen::VectorXd& K, Eigen::VectorXd& H)
{
    double maxGauss = _computeGaussCurvature(mesh, K);
    double maxMean = _computeMeanCurvature(mesh, H);
    
    // normalize gauss and mean curvature
    for (VertexIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        K(v->index) /= maxGauss;
        H(v->index) /= maxMean;
    }
}

void _buildSimpleAverager(Mesh *mesh, Eigen::SparseMatrix<double>& L)
{
    std::vector<Eigen::Triplet<double>> LTriplet;
    
    for (VertexCIter v = mesh->vertices.begin(); v != mesh->vertices.end(); v++) {
        HalfEdgeCIter he = v->he;
        double degree = v->degree();
        do {
            LTriplet.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index,
                                                      1.0/degree));
            
            he = he->flip->next;
        } while (he != v->he);
    }
    
    L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}



void KeypointDetectionBundle::computeEig(int K)
{
    int v = (int)mesh.vertices.size();
        
    // build adjacency operator
    Eigen::SparseMatrix<double> W(v, v);
    _buildAdjacency(&mesh, W);
        
    // build area matrix
    Eigen::SparseMatrix<double> A(v, v);
    _buildAreaMatrix(&mesh, A, v);
        
    //compute eigenvectors and eigenvalues
    Spectra::SparseSymMatProd<double> opW(W);
    Spectra::SparseCholesky<double> opA(A);

    // K 재설정
    int _K = (v / 100) * 100 / 2;
    K = std::min (_K, K);
    
    /*Spectra::SymGEigsSolver<double,
    Spectra::SMALLEST_MAGN,
    Spectra::SparseSymMatProd<double>,
    Spectra::SparseCholesky<double>,
    Spectra::GEIGS_CHOLESKY> geigs(&opW, &opA, K, 2*K);
        
    geigs.init();
    geigs.compute();
        
    if (geigs.info() == Spectra::SUCCESSFUL) {
         evals = geigs.eigenvalues();
         evecs = geigs.eigenvectors();
            
    } else {
        std::cout << "Eigen computation failed" << std::endl;
    }*/

    Spectra::SymGEigsSolver<
    Spectra::SparseSymMatProd<double>,
    Spectra::SparseCholesky<double>,
    Spectra::GEigsMode::Cholesky> geigs(opW, opA, K, 2*K);
        
    geigs.init();
    geigs.compute();
        
    if (geigs.info() == Spectra::CompInfo::Successful) {
        evals = geigs.eigenvalues();
        evecs = geigs.eigenvectors();
            
    } else {
        std::cout << "Eigen computation failed" << std::endl;
    }
        
    Eigen::MatrixXd err = W*evecs - A*evecs*evals.asDiagonal();
    std::cout << "||Lx - λAx||_inf = " << err.array().abs().maxCoeff() << std::endl;
    
}

void KeypointDetectionBundle::computeHks()
{
    const int K = (int)evals.size();
    const double ln = 4*log(10);
    const double tmin = ln/evals(0);
    const double step = (ln/evals(K-2) - tmin) / N;
    
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->descriptor = Eigen::VectorXd::Zero(N);
        Eigen::VectorXd C = Eigen::VectorXd::Zero(N);
        
        for (int j = K-2; j >= 0; j--) {
            double phi2 = evecs(v->index, j)*evecs(v->index, j);
            double t = tmin;
            double factor = 0.5;
            
            for (int i = 0; i < N; i++) {
                double exponent = exp(-evals(j)*t);
                v->descriptor(i) += phi2*exponent;
                C(i) += exponent;
                t += factor*step;
                
                factor += 0.1;
            }
        }
        
        // normalize
        for (int i = 0; i < N; i++) {
            v->descriptor(i) /= C(i);
        }
    }
}

void KeypointDetectionBundle::computeCurve()
{
	std::vector<int> smoothLevels = {0, 1, 5, 20, 50};
	 
	// compute the mean and gaussian curvature values
	int vsize = (int)mesh.vertices.size();
	Eigen::VectorXd K(vsize);
	Eigen::VectorXd H(vsize);
	_computeCurvatures(&mesh, K, H);

	// allocate space for the descriptors
	for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		v->descriptor = Eigen::VectorXd::Zero(N);
	}

	// compute a shifted-laplacian matrix for taking averages
	Eigen::SparseMatrix<double> avgM(vsize, vsize);
	_buildSimpleAverager(&mesh, avgM);

	// for each of the smoothing levels, smooth an appropriate number of times, then save the descriptor
	int smoothingStepsCompleted = 0;
	int iLevel = 0;
	for (int smoothLevel : smoothLevels) {
		// smooth as needed
		while (smoothingStepsCompleted < smoothLevel) {
			K = avgM * K;
			H = avgM * H;

			smoothingStepsCompleted++;
		}
        
		// save
		for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			v->descriptor(iLevel + 0) = K(v->index);
			v->descriptor(iLevel + 1) = H(v->index);
		}
		
		iLevel += 2;
	}
}

void KeypointDetectionBundle::normalize()
{
	int n = (int)mesh.vertices[0].descriptor.size();
	for (int i = 0; i < n; i++) {
		// compute min and max
		double min = 0.0, max = 0.0;
		for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			min = std::min(min, v->descriptor(i));
			max = std::max(max, v->descriptor(i));
		}
        
		// normalize
		for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			v->descriptor(i) = (v->descriptor(i) - min) / (max - min);
		}
	}
}

bool _IsLocalMaxima(int query_idx,
				   const std::vector<int>& indices,
				   const std::vector<double>& third_eigen_values) {
	for (const auto& idx : indices) {
		if (query_idx == idx)
			continue;
		
		if (third_eigen_values[query_idx] < third_eigen_values[idx]) {
			return false;
		}
	}
	return true;
}

double _ComputeModelResolution(const std::vector<Eigen::Vector3d>& points,
							  const KDTreeFlann& kdtree) {
	std::vector<int> indices(2);
	std::vector<double> distances(2);
	double resolution = 0.0;

	for (const auto& point : points) {
		if (kdtree.SearchKNN(point, 2, indices, distances) != 0) {
			resolution += std::sqrt(distances[1]);
		}
	}
	resolution /= points.size();

	indices.clear();
	std::vector<int>().swap(indices);

	distances.clear();
	std::vector<double>().swap(distances);
	
	return resolution;
}

template <typename IdxType>
Eigen::Matrix3d _ComputeCovariance(const std::vector<Eigen::Vector3d> &points,
								  const std::vector<IdxType> &indices)
{
	if (indices.empty()) {
		return Eigen::Matrix3d::Identity();
	}

	// TODO 매트릭스 메모리 누수 확인
	Eigen::Matrix3d covariance;
	Eigen::Matrix<double, 9, 1> cumulants;
	cumulants.setZero();
	for (const auto &idx : indices) {
		const Eigen::Vector3d &point = points[idx];
		cumulants(0) += point(0);
		cumulants(1) += point(1);
		cumulants(2) += point(2);
		cumulants(3) += point(0) * point(0);
		cumulants(4) += point(0) * point(1);
		cumulants(5) += point(0) * point(2);
		cumulants(6) += point(1) * point(1);
		cumulants(7) += point(1) * point(2);
		cumulants(8) += point(2) * point(2);
	}
	cumulants /= (double)indices.size();
	covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
	covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
	covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
	covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
	covariance(1, 0) = covariance(0, 1);
	covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
	covariance(2, 0) = covariance(0, 2);
	covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
	covariance(2, 1) = covariance(1, 2);

	return covariance;
}

std::vector<int> KeypointDetectionBundle::ComputeISSKeypoints(
	const std::vector<Eigen::Vector3d>& input, double salient_radius, double non_max_radius, double gamma_21,
	double gamma_32, int min_neighbors)
{
    if (input.empty())
    {
        //utility::LogWarning("[ComputeISSKeypoints] Input PointCloud is empty!");
        return std::vector<int>{};
    }
	
	Eigen::MatrixXd _input = Eigen::MatrixXd::Zero (3, input.size());
	// TODO vector vector3d -> vectorxd로 변환할 것
	for (int _i = 0; _i < input.size(); _i++)
	{
		// _input.setConstant(0, _i, input[_i].x());
		// _input.setConstant(1, _i, input[_i].y());
		// _input.setConstant(2, _i, input[_i].z());
		_input (0, _i) = input[_i][0];
		_input (1, _i) = input[_i][1];
		_input (2, _i) = input[_i][2];
	}
	
    if (!kdtree.SetMatrixData(_input))
    {
    	return std::vector<int>{};
    }
	
    if (salient_radius == 0.0 || non_max_radius == 0.0) {
        const double resolution = _ComputeModelResolution(input, kdtree);
        salient_radius = 6 * resolution;
        non_max_radius = 4 * resolution;
        // utility::LogDebug(
        //         "[ComputeISSKeypoints] Computed salient_radius = {}, "
        //         "non_max_radius = {} from input model",
        //         salient_radius, non_max_radius);
    }
	eigenValues.clear();
	vector<double>().swap(eigenValues);

//#pragma omp parallel for schedule(static) shared(third_eigen_values)
    for (int i = 0; i < (int)input.size(); i++) {
        std::vector<int> indices;
        std::vector<double> dist;
        int nb_neighbors =
                kdtree.SearchRadius(input[i], salient_radius, indices, dist);
        if (nb_neighbors < min_neighbors) {
        	eigenValues.push_back(0);
            continue;
        }

        Eigen::Matrix3d cov = _ComputeCovariance(input, indices);
        if (cov.isZero()) {
        	eigenValues.push_back(0);
            continue;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        double e1c = solver.eigenvalues()[2];
        double e2c = solver.eigenvalues()[1];
        double e3c = solver.eigenvalues()[0];

        if ((e2c / e1c) < gamma_21 && e3c / e2c < gamma_32) {
            eigenValues.push_back(e3c);
        }
    	else
    	{
    		eigenValues.push_back(0);
    	}

    	cov.Zero();
    	
    	//cout <<  (e1c) << ", " << e2c << ", " << e3c << std::endl;

    	indices.clear ();
    	std::vector<int> ().swap(indices);
    	
    	dist.clear();
    	std::vector<double> ().swap(dist);
    }

    std::vector<int> kp_indices = std::vector<int> ();
    //kp_indices.reserve(input.size());
//#pragma omp parallel for schedule(static) shared(kp_indices)
    for (int i = 0; i < (int)input.size(); i++) {
        if (eigenValues[i] > 0.0) {
            std::vector<int> nn_indices;
            std::vector<double> dist;
            int nb_neighbors = kdtree.SearchRadius(input[i], non_max_radius,
                                                   nn_indices, dist);

            if (nb_neighbors >= min_neighbors && _IsLocalMaxima(i, nn_indices, eigenValues)) {
                kp_indices.push_back (i);
            }

        	nn_indices.clear ();
        	std::vector<int> ().swap(nn_indices);
    	
        	dist.clear();
        	std::vector<double> ().swap(dist);
        }
    }
    
	

    // utility::LogDebug("[ComputeISSKeypoints] Extracted {} keypoints",
    //                   kp_indices.size());
    return kp_indices;
}

/*
EVertexType Descriptor_ISS::GetVertexType(int index, const int depth, const float dotFlat0, const float dotFlat1)
{
	EVertexType ret = EVertexType::NONE;

	std::queue<int> queue;
	std::unordered_map<int, bool> visited;
    
	// enqueue
	queue.push(index);
	queue.push(-1);
	visited[index] = true;
	int levels = 0;

	std::vector<int> neighborIndices = std::vector<int>();
    
	// 주변 이웃 인덱스 구하기
	while (!queue.empty())
	{
		int v = queue.front();
		queue.pop();
	    
		if (v == -1)
		{
			levels++;
			queue.push(-1);
			if (queue.front() == -1 || levels == depth) break;
		} 
		else
		{
			for (int u : meshData->neighbors[v])
			{
				if (!visited[u])
				{
					neighborIndices.push_back(u);

					visited[u] = true;
					queue.push (u);
				}
			}
		}
	}

	// 초기화
	while (!queue.empty())
		queue.pop();
	
	visited.clear();
	std::unordered_map<int, bool>().swap(visited);
	
	// 얻은 주변 인덱스를 이용하여 Type 계산
	if (neighborIndices.size() == 0)
		return EVertexType::NONE;

	FVector neighboursPos_sum = FVector::Zero();
	FVector neighboursPos_avg = FVector::Zero();

	for (int i = 0; i != neighborIndices.size(); i++)
		neighboursPos_sum += FVector(meshData->positions[neighborIndices[i]].x(), meshData->positions[neighborIndices[i]].y(), meshData->positions[neighborIndices[i]].z());

	neighboursPos_avg = neighboursPos_sum / (1.0 * neighborIndices.size());

	FVector direction_self = FVector(meshData->normals[index].x(), meshData->normals[index].y(), meshData->normals[index].z());
	FVector direction_avg = FVector(meshData->positions[index].x(), meshData->positions[index].y(), meshData->positions[index].z()) - neighboursPos_avg;
	direction_avg = direction_avg / FVector::Distance(FVector::Zero(), direction_avg);

	float dotP = FVector::DotProduct(direction_self, direction_avg);
	
	if (dotFlat0 < dotP && dotP <= dotFlat1)
	{
		ret = EVertexType::VERTEX_FLAT;
	}
	else
	{
		if (dotP > 0)
		{
			ret = EVertexType::VERTEX_BUMP;
		}
		else
		{
			ret = EVertexType::VERTEX_SINK;
		}
	}

	neighborIndices.clear();
	std::vector<int>().swap(neighborIndices);

	return ret;
}

EVertexNormalType Descriptor_ISS::GetVertexNormalType(int index, const float dotUp, const float dotDown)
{
	FVector nor = FVector(meshData->normals[index].x(), meshData->normals[index].y(), meshData->normals[index].z());
    
	// 노멀 구분하기  
	float dot = FVector::DotProduct(nor, FVector::UpVector);

	if (dot > dotUp)
	{
		return EVertexNormalType::VERTEX_UP;
	}
	
	if (dot < dotDown)
	{
		return EVertexNormalType::VERTEX_DOWN;
	}

	return EVertexNormalType::VERTEX_PARALLEL;
}*/

// ================================================================================================================================


void KeypointDetectionBundle::computeSaliency()
{
	 const int levels = 5;
    
    int count = (int)mesh.vertices.size();
    double minSaliency = INFINITY;
    double maxSaliency = -INFINITY;
    std::vector<double> levelSaliencies(count);
    
    // 1: compute mean curvature
    computeMeanCurvature();
    
    // 2: initialize and compute extent
    BoundingBox bbox;
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        bbox.expandToInclude(v->position);
    }
	// TODO 수정
	// 길이의 제곱
    double extent = 0.003 * 0.003 * bbox.extent.squaredNorm();
    
    // 3
	// {2, 3, 4, 5, 6}
    for (int i = 0; i < levels; i++) {
        
        // compute level saliencies
        double sumSaliency = 0.0;
        double distance2 = (i+2)*(i+2)*extent;
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            
            double weightedCurvature1 = v->computeWeightedCurvature(v, distance2);
            double weightedCurvature2 = v->computeWeightedCurvature(v, 4*distance2);
            
            levelSaliencies[v->index] = std::abs(weightedCurvature1 - weightedCurvature2);
            sumSaliency += levelSaliencies[v->index];
        }
        
        // normalize
        double maxLevelSaliency = -INFINITY;
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            levelSaliencies[v->index] /= sumSaliency;
            if (maxLevelSaliency < levelSaliencies[v->index]) maxLevelSaliency = levelSaliencies[v->index];
        }
        
        // compute mean of local maxima
        double peaks = 0.0;
        double meanLocalMaxSaliency = 0.0;
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            if (levelSaliencies[v->index] != maxLevelSaliency && v->isPeakSaliency(v, levelSaliencies)) {
                meanLocalMaxSaliency += levelSaliencies[v->index];
                peaks += 1.0;
            }
        }
        meanLocalMaxSaliency /= peaks;
        
        // apply non-linear suppression operator to level saliency
        double suppressionFactor = pow(maxLevelSaliency - meanLocalMaxSaliency, 2);
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            v->saliency += levelSaliencies[v->index] * suppressionFactor;
            
            if (i+1 == levels) {
                if (v->saliency < minSaliency) minSaliency = v->saliency;
                if (maxSaliency < v->saliency) maxSaliency = v->saliency;
            }
        }
    }
    
    // 4: scale between 0 and 1
    double dSaliency = maxSaliency - minSaliency;
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->saliency = (v->saliency - minSaliency) / dSaliency;
    }
}

void KeypointDetectionBundle::buildLaplacian(Eigen::SparseMatrix<double>& L) const
{
	std::vector<Eigen::Triplet<double>> LTriplet;
    
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        
		HalfEdgeCIter he = v->he;
		double dualArea = v->dualArea();
		double sumCoefficients = 0.0;
		do {
			// (cotA + cotB) / 2A
			double coefficient = 0.5 * (he->cotan() + he->flip->cotan()) / dualArea;
			sumCoefficients += coefficient;
            
			LTriplet.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, coefficient));
            
			he = he->flip->next;
		} while (he != v->he);
        
		LTriplet.push_back(Eigen::Triplet<double>(v->index, v->index, -sumCoefficients));
	}
    
	L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

void KeypointDetectionBundle::computeMeanCurvature()
{
	int _v = (int)mesh.vertices.size();
	Eigen::SparseMatrix<double> L(_v, _v);
	buildLaplacian(L);
    
	Eigen::MatrixXd x;
	x.resize(_v, 3);
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		x.row(v->index) = v->position;
	}
	x = L * x;
    
	for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		v->meanCurvature = 0.5 * x.row(v->index).norm();
	}
}