#pragma once


namespace pcl
{
template<typename PointT>
class PointCloud;

class PolygonMesh;
}

namespace sq
{
  template<typename T>
  struct SuperquadricParameters;

  /**
   * Naive superquadric sampling (i.e., uniformly sample the parameter space)
   */
  template <typename PointT, typename Scalar>
  class SuperquadricSampling
  {
  public:
    SuperquadricSampling ();

    inline void
    setParameters (const SuperquadricParameters<Scalar> &params)
    { params_ = params; }

    inline void
    setSampleCount (const int eta_samples,
                    const int mu_samples)
    { eta_samples_ = eta_samples;
      mu_samples_ = mu_samples; }


    void
    generatePointCloud (pcl::PointCloud<PointT> &cloud);

    void
    generateMesh (pcl::PolygonMesh &mesh);


  protected:
    SuperquadricParameters<Scalar> params_;
    int eta_samples_;
    int mu_samples_;
  };
}


#include "impl/sample_superquadric.hpp"
