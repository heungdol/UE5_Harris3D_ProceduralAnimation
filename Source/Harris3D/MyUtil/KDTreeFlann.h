#pragma once

#include <ThirdParty/Eigen/Eigen/Core>
#include <memory>
#include <vector>

#include "nanoflann.h"

namespace nanoflann {
    struct metric_L2;
    template <class MatrixType, int DIM, class Distance, bool row_major>
    struct KDTreeEigenMatrixAdaptor;
} 

class KDTreeSearchParam {
public:
    /// \enum SearchType
    ///
    /// \brief Specifies the search type for the search.
    enum class SearchType {
        Knn = 0,
        Radius = 1,
        Hybrid = 2,
    };

public:
    virtual ~KDTreeSearchParam() {}

protected:
    KDTreeSearchParam(SearchType type) : search_type_(type) {}

public:
    /// Get the search type (KNN, Radius, Hybrid) for the search parameter.
    SearchType GetSearchType() const { return search_type_; }

private:
    SearchType search_type_;
};

/// \class KDTreeSearchParamKNN
///
/// \brief KDTree search parameters for pure KNN search.
class KDTreeSearchParamKNN : public KDTreeSearchParam {
public:
    /// \brief Default Constructor.
    ///
    /// \param knn Specifies the knn neighbors that will searched. Default
    /// is 30.
    KDTreeSearchParamKNN(int knn = 30)
        : KDTreeSearchParam(SearchType::Knn), knn_(knn) {}

public:
    /// Number of the neighbors that will be searched.
    int knn_;
};

/// \class KDTreeSearchParamRadius
///
/// \brief KDTree search parameters for pure radius search.
class KDTreeSearchParamRadius : public KDTreeSearchParam {
public:
    /// \brief Default Constructor.
    ///
    /// \param radius Specifies the radius of the search.
    KDTreeSearchParamRadius(double radius)
        : KDTreeSearchParam(SearchType::Radius), radius_(radius) {}

public:
    /// Search radius.
    double radius_;
};

/// \class KDTreeSearchParamHybrid
///
/// \brief KDTree search parameters for hybrid KNN and radius search.
class KDTreeSearchParamHybrid : public KDTreeSearchParam {
public:
    /// \brief Default Constructor.
    ///
    /// \param radius Specifies the radius of the search.
    /// \param max_nn Specifies the max neighbors to be searched.
    KDTreeSearchParamHybrid(double radius, int max_nn)
        : KDTreeSearchParam(SearchType::Hybrid),
          radius_(radius),
          max_nn_(max_nn) {}

public:
    /// Search radius.
    double radius_;
    /// At maximum, max_nn neighbors will be searched.
    int max_nn_;
};

class KDTreeFlann
{
public:
    /// \brief Default Constructor.
    KDTreeFlann();
    /// \brief Parameterized Constructor.
    ///
    /// \param data Provides set of data points for KDTree construction.
    KDTreeFlann(const Eigen::MatrixXd &data);
    /// \brief Parameterized Constructor.
    ///
    /// \param geometry Provides geometry from which KDTree is constructed.
    //KDTreeFlann(const Geometry &geometry);
    /// \brief Parameterized Constructor.
    ///
    /// \param feature Provides a set of features from which the KDTree is
    /// constructed.
    //KDTreeFlann(const pipelines::registration::Feature &feature);
    ~KDTreeFlann();
    KDTreeFlann(const KDTreeFlann &) = delete;
    KDTreeFlann &operator=(const KDTreeFlann &) = delete;

public:
    /// Sets the data for the KDTree from a matrix.
    ///
    /// \param data Data points for KDTree Construction.
    bool SetMatrixData(const Eigen::MatrixXd &data);
    /// Sets the data for the KDTree from geometry.
    ///
    /// \param geometry Geometry for KDTree Construction.
    //bool SetGeometry(const Geometry &geometry);
    /// Sets the data for the KDTree from the feature data.
    ///
    /// \param feature Set of features for KDTree construction.
    //bool SetFeature(const pipelines::registration::Feature &feature);

    template <typename T>
    int Search(const T &query,
               const KDTreeSearchParam &param,
               std::vector<int> &indices,
               std::vector<double> &distance2) const;

    template <typename T>
    int SearchKNN(const T &query,
                  int knn,
                  std::vector<int> &indices,
                  std::vector<double> &distance2) const;

    template <typename T>
    int SearchRadius(const T &query,
                     double radius,
                     std::vector<int> &indices,
                     std::vector<double> &distance2) const;

    template <typename T>
    int SearchHybrid(const T &query,
                     double radius,
                     int max_nn,
                     std::vector<int> &indices,
                     std::vector<double> &distance2) const;

private:
    /// \brief Sets the KDTree data from the data provided by the other methods.
    ///
    /// Internal method that sets all the members of KDTree by data provided by
    /// features, geometry, etc.
    bool SetRawData(const Eigen::Map<const Eigen::MatrixXd> &data);

protected:
    using KDTree_t = nanoflann::KDTreeEigenMatrixAdaptor<
            Eigen::Map<const Eigen::MatrixXd>,
            -1,
            nanoflann::metric_L2,
            false>;

    std::vector<double> data_;
    std::unique_ptr<Eigen::Map<const Eigen::MatrixXd>> data_interface_;
    std::unique_ptr<KDTree_t> nanoflann_index_;
    size_t dimension_ = 0;
    size_t dataset_size_ = 0;

};
