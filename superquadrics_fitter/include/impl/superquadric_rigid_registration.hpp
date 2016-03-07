#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

#include <ceres/ceres.h>

#include "superquadric_rigid_registration.h"
#include "superquadric_formulas.h"


////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar>
sq::SuperquadricRigidRegistration<PointT, MatScalar>::SuperquadricRigidRegistration ()
  : pre_align_ (true)
  , pre_align_axis_ (2)
{
  init_transform_ = Eigen::Matrix<MatScalar, 4, 4>::Identity ();
}


////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar> void
sq::SuperquadricRigidRegistration<PointT, MatScalar>::preAlign (Eigen::Matrix<MatScalar, 4, 4> &transformation_prealign)
{
  /// Compute the centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid (*input_, centroid);
  Eigen::Matrix<MatScalar, 4, 4> transformation_centroid (Eigen::Matrix<MatScalar, 4, 4>::Identity ());
  transformation_centroid (0, 3) = - centroid (0);
  transformation_centroid (1, 3) = - centroid (1);
  transformation_centroid (2, 3) = - centroid (2);

  std::cout << "centroid: " << centroid << std::endl;
  std::cout << "from " << input_->size () << " points" << std::endl;

  /// Compute the PCA
  pcl::PCA<PointT> pca;
  pca.setInputCloud (input_);
  Eigen::Vector3f eigenvalues = pca.getEigenValues ();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors ();

  std::cout << "eigenvectors:\n" << eigenvectors << std::endl;

  /// Align the first PCA axis with the prealign axis
  Eigen::Vector3f vec_aux = eigenvectors.row (0);
  eigenvectors.row (0) = eigenvectors.row (pre_align_axis_);
  eigenvectors.row (pre_align_axis_) = vec_aux;

  float aux_ev = eigenvalues (0);
  eigenvalues (0) = eigenvalues (pre_align_axis_);
  eigenvalues (pre_align_axis_) = aux_ev;


  Eigen::Matrix<MatScalar, 4, 4> transformation_pca (Eigen::Matrix<MatScalar, 4, 4>::Identity ());
  transformation_pca (0, 0) = eigenvectors (0, 0);
  transformation_pca (1, 0) = eigenvectors (0, 1);
  transformation_pca (2, 0) = eigenvectors (0, 2);

  transformation_pca (0, 1) = eigenvectors (1, 0);
  transformation_pca (1, 1) = eigenvectors (1, 1);
  transformation_pca (2, 1) = eigenvectors (1, 2);

  transformation_pca (0, 2) = eigenvectors (2, 0);
  transformation_pca (1, 2) = eigenvectors (2, 1);
  transformation_pca (2, 2) = eigenvectors (2, 2);

  transformation_prealign = transformation_pca * transformation_centroid;

  std::cout << "pre-align transformation:\n" << transformation_prealign << std::endl;
}


////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar> double
sq::SuperquadricRigidRegistration<PointT, MatScalar>::fit (Eigen::Matrix<MatScalar, 4, 4> &transform)
{
  Eigen::Matrix<MatScalar, 4, 4> transformation_prealign (Eigen::Matrix<MatScalar, 4, 4>::Identity ());

  if (pre_align_)
    preAlign (transformation_prealign);
  else
    transformation_prealign = init_transform_;


  input_prealigned_.reset (new Cloud ());
  pcl::transformPointCloud (*input_, *input_prealigned_, transformation_prealign);

  double xvec[6];
  xvec[0] = xvec[1] = xvec[2] = xvec[3] = xvec[4] = xvec[5] = 0.;


  printf ("xvec before = ");
  for (size_t i = 0; i < 6; ++i)
    printf ("%f ", xvec[i]);
  printf ("\n");


  ceres::Problem problem;
  for (size_t p_i = 0; p_i < input_prealigned_->size (); ++p_i)
  {
    const PointT &point = (*input_prealigned_)[p_i];
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<SuperquadricCostFunctor, 1, 6> (new SuperquadricCostFunctor (point, e1_, e2_, a_, b_, c_));
    problem.AddResidualBlock (cost_function, NULL, xvec);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 50;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;


  ceres::Solver::Summary summary;
  ceres::Solve (options, &problem, &summary);

  /// If we did not converge, return infinite error
  if (summary.termination_type == ceres::NO_CONVERGENCE)
      //summary.termination_type == ceres:: ||
      //summary.termination_type == ceres::DID_NOT_RUN)
  {
    PCL_ERROR ("Did not converge.\n");
//    std::cout << summary.FullReport () << std::endl;
    transform = transformation_prealign;
    return (std::numeric_limits<double>::infinity ());
  }

  std::cout << summary.BriefReport () << std::endl;


  printf ("x = ");
  for (size_t i = 0; i < 6; ++i)
    printf ("%f ", xvec[i]);
  printf ("\n");


  transform.setZero ();
  transform (0, 3) = xvec[0];
  transform (1, 3) = xvec[1];
  transform (2, 3) = xvec[2];
  transform (3, 3) = 1.;
  transform.block (0, 0, 3, 3) = Eigen::AngleAxis<MatScalar> (xvec[3], Eigen::Matrix<MatScalar, 3, 1>::UnitZ ()) *
                                 Eigen::AngleAxis<MatScalar> (xvec[4], Eigen::Matrix<MatScalar, 3, 1>::UnitX ()) *
                                 Eigen::AngleAxis<MatScalar> (xvec[5], Eigen::Matrix<MatScalar, 3, 1>::UnitZ ()).matrix ();

  transform = transform * transformation_prealign;


  MatScalar final_error = computeSuperQuadricError<PointT, MatScalar> (input_,
                                                                       e1_, e2_, a_, b_, c_,
                                                                       transform);

  return (final_error);
}


////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename MatScalar>
template <typename T> bool
sq::SuperquadricRigidRegistration<PointT, MatScalar>::SuperquadricCostFunctor::operator () (const T* const xvec, T* residual) const
{
//  PCL_ERROR ("Cost functor called\n");
  Eigen::Matrix<T, 4, 4> transformation;
  transformation.setZero ();
  transformation (0, 3) = xvec[0];
  transformation (1, 3) = xvec[1];
  transformation (2, 3) = xvec[2];
  transformation (3, 3) = T (1.);
  transformation.block (0, 0, 3, 3) = Eigen::AngleAxis<T> (xvec[3], Eigen::Matrix<T, 3, 1>::UnitZ ()) *
                                      Eigen::AngleAxis<T> (xvec[4], Eigen::Matrix<T, 3, 1>::UnitX ()) *
                                      Eigen::AngleAxis<T> (xvec[5], Eigen::Matrix<T, 3, 1>::UnitZ ()).matrix ();


  Eigen::Matrix<T, 4, 1> xyz (T (point_.x), T (point_.y), T (point_.z), T (1.));
  Eigen::Matrix<T, 4, 1> xyz_tr = transformation * xyz;

//  std::cout << "xyz_tr: " << xyz_tr[0] << " " << xyz_tr[1] << " " << xyz_tr[2] << std::endl;

  residual[0] = superquadric_function<T> (xyz_tr[0], xyz_tr[1], xyz_tr[2], T (e1_), T (e2_), T (a_), T (b_), T (c_));


  /// TODO do something if the residual is inf, set it to something large 1e20 or so
  //  if (residual[0])

//  std::cout << residual[0] << std::endl;


  return (true);
}
