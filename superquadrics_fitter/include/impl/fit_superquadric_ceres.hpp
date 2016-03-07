#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

#include <ceres/ceres.h>

#include "fit_superquadric_ceres.h"
#include "superquadric_formulas.h"


////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar>
sq::SuperquadricFittingCeres<PointT, MatScalar>::SuperquadricFittingCeres ()
  : pre_align_ (true)
  , pre_align_axis_ (2)
{
  init_parameters_.a = 1.;
  init_parameters_.b = 1.;
  init_parameters_.c = 1.;
  init_parameters_.e1 = 1.;
  init_parameters_.e2 = 1.;
  init_parameters_.transform = Eigen::Matrix4d::Identity ();
}


////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar> void
sq::SuperquadricFittingCeres<PointT, MatScalar>::preAlign (Eigen::Matrix<MatScalar, 4, 4> &transformation_prealign,
                                                           Eigen::Matrix<MatScalar, 3, 1> &variances)
{
  /// Compute the centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid (*input_, centroid);
  Eigen::Matrix<MatScalar,4,1> tf_centroid;
  tf_centroid (0,0) = centroid (0);
  tf_centroid (1,0) = centroid (1);
  tf_centroid (2,0) = centroid (2);
  tf_centroid (3,0) = 1;

  //std::cout << "centroid: " << centroid << std::endl;
  //=std::cout << "from " << input_->size () << " points" << std::endl;

  /// Compute the PCA
  pcl::PCA<PointT> pca;
  pca.setInputCloud (input_);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigvectors_float = pca.getEigenVectors();
  Eigen::Matrix<MatScalar,3,3> eigenvectors = eigvectors_float.cast<MatScalar>();

  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

  transformation_prealign = Eigen::Matrix<MatScalar,4,4>::Identity();
  transformation_prealign.template block<3,3>(0,0) = eigenvectors.transpose();
  transformation_prealign.template block<3,1>(0,3) = -1.f * (transformation_prealign.template block<3,3>(0,0) * tf_centroid.template block<3,1>(0,0));

  /// Set the variances
  eigenvalues /= static_cast<float> (input_->size ());
  variances (0) = sqrt (eigenvalues (0));
  variances (1) = sqrt (eigenvalues (1));
  variances (2) = sqrt (eigenvalues (2));

  //std::cout << "variances:\n" << variances << std::endl << eigenvalues << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////
//template<typename PointT, typename MatScalar> void
//sq::SuperquadricFittingCeres<PointT, MatScalar>::preAlign (Eigen::Matrix<MatScalar, 4, 4> &transformation_prealign,
//                                                           Eigen::Matrix<MatScalar, 3, 1> &variances)
//{
//  /// Compute the centroid
//  Eigen::Vector4d centroid;
//  pcl::compute3DCentroid (*input_, centroid);
//  Eigen::Matrix<MatScalar, 4, 4> transformation_centroid (Eigen::Matrix<MatScalar, 4, 4>::Identity ());
//  transformation_centroid (0, 3) = - centroid (0);
//  transformation_centroid (1, 3) = - centroid (1);
//  transformation_centroid (2, 3) = - centroid (2);

//  //std::cout << "centroid: " << centroid << std::endl;
//  //=std::cout << "from " << input_->size () << " points" << std::endl;

//  /// Compute the PCA
//  pcl::PCA<PointT> pca;
//  pca.setInputCloud (input_);
//  Eigen::Vector3f eigenvalues = pca.getEigenValues ();
//  Eigen::Matrix3f eigenvectors = pca.getEigenVectors ();

//  //std::cout << "eigenvectors:\n" << eigenvectors << std::endl;

//  /// Align the first PCA axis with the prealign axis
//  Eigen::Vector3f vec_aux = eigenvectors.row (0);
//  eigenvectors.row (0) = eigenvectors.row (pre_align_axis_);
//  eigenvectors.row (pre_align_axis_) = vec_aux;

//  float aux_ev = eigenvalues (0);
//  eigenvalues (0) = eigenvalues (pre_align_axis_);
//  eigenvalues (pre_align_axis_) = aux_ev;


//  Eigen::Matrix<MatScalar, 4, 4> transformation_pca (Eigen::Matrix<MatScalar, 4, 4>::Identity ());
//  transformation_pca (0, 0) = eigenvectors (0, 0);
//  transformation_pca (1, 0) = eigenvectors (0, 1);
//  transformation_pca (2, 0) = eigenvectors (0, 2);

//  transformation_pca (0, 1) = eigenvectors (1, 0);
//  transformation_pca (1, 1) = eigenvectors (1, 1);
//  transformation_pca (2, 1) = eigenvectors (1, 2);

//  transformation_pca (0, 2) = eigenvectors (2, 0);
//  transformation_pca (1, 2) = eigenvectors (2, 1);
//  transformation_pca (2, 2) = eigenvectors (2, 2);

//  transformation_prealign = transformation_pca * transformation_centroid;

//  //std::cout << "pre-align transformation:\n" << transformation_prealign << std::endl;

//  /// Set the variances
//  eigenvalues /= static_cast<float> (input_->size ());
//  variances (0) = sqrt (eigenvalues (0));
//  variances (1) = sqrt (eigenvalues (1));
//  variances (2) = sqrt (eigenvalues (2));

//  //std::cout << "variances:\n" << variances << std::endl << eigenvalues << std::endl;
//}



////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename MatScalar>
double
sq::SuperquadricFittingCeres<PointT, MatScalar>::fit (SuperquadricParameters<MatScalar> &parameters)
{
  Eigen::Matrix<MatScalar, 4, 4> transformation_prealign (Eigen::Matrix<MatScalar, 4, 4>::Identity ());
  Eigen::Matrix<MatScalar, 3, 1> variances;
  variances (0) = variances (1) = variances (2) = static_cast <MatScalar> (1.);

  if (pre_align_)
  {
    preAlign (transformation_prealign, variances);
    input_prealigned_.reset (new Cloud ());
    pcl::transformPointCloud (*input_, *input_prealigned_, transformation_prealign);
  }else{
      input_prealigned_.reset (new Cloud ());
      pcl::copyPointCloud(*input_,*input_prealigned_);
  }



  ceres::Problem problem;

  double xvec[11];
  xvec[0] = xvec[1] = 1.;
  xvec[2] = variances (0) * 3.;
  xvec[3] = variances (1) * 3.;
  xvec[4] = variances (2) * 3.;
  xvec[5] = xvec[6] = xvec[7] = xvec[8] = xvec[9] = xvec[10] = 0.;

  double max_abs_scale = 0.12;
  double max_translation = 0.02;
  double min_rotation = -0.375*M_PI;
  double max_rotation = 0.375*M_PI;
  double min_e1 = 0.1;
  double min_e2 = 0.1;
  double max_e1 = 0.7;
  double max_e2 = 0.7;

//  double max_abs_scale = 0.1;
//  double max_translation = 0.02;
//  double min_rotation = -M_PI/4;
//  double max_rotation = M_PI/4;
//  double min_e1 = 0.1;
//  double min_e2 = 0.1;
//  double max_e1 = 0.7;
//  double max_e2 = 0.7;


  const double kDoubleMax = std::numeric_limits<double>::max();
  double lower_bounds[11] = {min_e1,min_e2,0,0,0,-max_translation,-max_translation,-max_translation,min_rotation,min_rotation,min_rotation};
  double upper_bounds[11] = {max_e1,max_e2,max_abs_scale,max_abs_scale,max_abs_scale,max_translation,max_translation,max_translation,max_rotation,max_rotation,max_rotation};


  for (size_t p_i = 0; p_i < input_prealigned_->size (); ++p_i)
  {
    PointT &point = (*input_prealigned_)[p_i];
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<SuperquadricCostFunctor, 1, 11> (new SuperquadricCostFunctor (point));
    //    ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<SuperquadricCostFunctor, ceres::CENTRAL, 1, 11> (new SuperquadricCostFunctor (point));
    problem.AddResidualBlock (cost_function, new ceres::SoftLOneLoss(1.0), xvec); //Other Loss functions are Null, HuberLoss, SoftLOneLoss
    //Other Loss functions are Null, HuberLoss, SoftLOneLoss

  }

  for (int i = 0; i < 11; ++i) {
    problem.SetParameterLowerBound(xvec, i, lower_bounds[i]);
    problem.SetParameterUpperBound(xvec, i, upper_bounds[i]);
  }

  ceres::Solver::Options options;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::DENSE_QR; //ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 4;
  options.parameter_tolerance = 1e-4;
  options.function_tolerance = 1e-4;
  options.gradient_tolerance = 1e-4;
  options.max_num_iterations = 100;


  ceres::Solver::Summary summary;
  ceres::Solve (options, &problem, &summary);

  /// If we did not converge, return infinite error
  if (summary.termination_type == ceres::NO_CONVERGENCE )
  {
    PCL_ERROR ("Did not converge.\n");
    return (std::numeric_limits<double>::infinity ());
  }

  std::cout << summary.BriefReport () << std::endl;

  printf ("x = ");
  for (size_t i = 0; i < 11; ++i)
    printf ("%f ", xvec[i]);
  printf ("\n");


  Eigen::Matrix<MatScalar, 4, 4> &transformation = parameters.transform;
  transformation.setZero ();
  transformation (0, 3) = xvec[5];
  transformation (1, 3) = xvec[6];
  transformation (2, 3) = xvec[7];
  transformation (3, 3) = 1.;
  transformation.block (0, 0, 3, 3) = Eigen::AngleAxis<MatScalar> (xvec[8], Eigen::Matrix<MatScalar, 3, 1>::UnitZ ()) *
                                      Eigen::AngleAxis<MatScalar> (xvec[9], Eigen::Matrix<MatScalar, 3, 1>::UnitX ()) *
                                      Eigen::AngleAxis<MatScalar> (xvec[10], Eigen::Matrix<MatScalar, 3, 1>::UnitZ ()).matrix ();

  cout << "Roll: " << xvec[8]*180/M_PI <<  " Pitch: " << xvec[9]*180/M_PI <<  " Yaw: " << xvec[10]*180/M_PI << endl;

  //clampParameters (xvec[0], xvec[1]);

  parameters.e1 = xvec[0];
  parameters.e2 = xvec[1];
  parameters.a = xvec[2];
  parameters.b = xvec[3];
  parameters.c = xvec[4];
  parameters.transform = Eigen::Matrix<MatScalar, 4, 4> (transformation);
  if(pre_align_) parameters.pre_align_transform = Eigen::Matrix<MatScalar, 4, 4> (transformation_prealign);

  MatScalar final_error = computeSuperQuadricError<PointT, MatScalar> (input_,
                                                                       xvec[0], xvec[1], xvec[2], xvec[3], xvec[4],
                                                                       transformation);


  return (final_error);
}


////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename MatScalar>
template <typename T>
bool
sq::SuperquadricFittingCeres<PointT, MatScalar>::SuperquadricCostFunctor::operator () (const T* const xvec, T* residual) const
{
  T e1 = xvec[0],
      e2 = xvec[1],
      a = xvec[2],
      b = xvec[3],
      c = xvec[4];
  Eigen::Matrix<T, 4, 4> transformation;
  transformation.setZero ();
  transformation (0, 3) = xvec[5];
  transformation (1, 3) = xvec[6];
  transformation (2, 3) = xvec[7];
  transformation (3, 3) = T (1.);
  transformation.block (0, 0, 3, 3) = Eigen::AngleAxis<T> (xvec[8], Eigen::Matrix<T, 3, 1>::UnitZ ()) *
                                      Eigen::AngleAxis<T> (xvec[9], Eigen::Matrix<T, 3, 1>::UnitX ()) *
                                      Eigen::AngleAxis<T> (xvec[10], Eigen::Matrix<T, 3, 1>::UnitZ ()).matrix ();

  Eigen::Matrix<T, 4, 1> xyz (T (point_.x), T (point_.y), T (point_.z), T (1.));
  Eigen::Matrix<T, 4, 1> xyz_tr = transformation * xyz;
  T op = Eigen::Matrix<T, 3, 1> (xyz_tr[0], xyz_tr[1], xyz_tr[2]).norm ();

  residual[0] = /*op */superquadric_function_scale_weighting<T> (xyz_tr[0], xyz_tr[1], xyz_tr[2], e1, e2, a, b, c);


  return (true);
}
