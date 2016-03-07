#include <fruit_rgbd.h>
#include <cloud_registration.h>


void pairAlignNDT (const PointCloud_::Ptr input_cloud, const PointCloud_::Ptr target_cloud, PointCloud_::Ptr output_cloud,Eigen::Matrix4f &final_transform)
{

	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	PointCloud_::Ptr filtered_cloud (new PointCloud_);
	pcl::ApproximateVoxelGrid<PointT_> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.01, 0.01, 0.01);
	approximate_voxel_filter.setInputCloud (input_cloud);
	approximate_voxel_filter.filter (*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size () << std::endl;

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<PointT_, PointT_> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	//ndt.setTransformationEpsilon (0.005);
	ndt.setTransformationEpsilon (1e-6);	

	// Setting maximum step size for More-Thuente line search.
	//ndt.setStepSize (0.001);
	ndt.setStepSize (0.005);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	//ndt.setResolution (0.01);
	ndt.setResolution (0.05);	

	// Setting max number of registration iterations.
	ndt.setMaximumIterations (500);

	// Setting point cloud to be aligned.
	ndt.setInputSource (filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);

	// Set initial alignment estimate found using robot odometry.
	//Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
	//Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
	//Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

	// Calculating required rigid transform to align the input cloud to the target cloud.
	PointCloud_::Ptr reg_result (new PointCloud_);
	ndt.align (*reg_result);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
	<< " score: " << ndt.getFitnessScore () << std::endl;

	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), targetToSource;
	Ti = ndt.getFinalTransformation ();
	targetToSource = Ti.inverse();

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*target_cloud, *output_cloud, targetToSource);
	final_transform = targetToSource;
}




////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */


void pairAlignICP (const PointCloud_::Ptr cloud_src, const PointCloud_::Ptr cloud_tgt, PointCloud_::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)

//From http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php
//void pairAlignICP (const PointCloud_::Ptr cloud_src, const PointCloud_::Ptr cloud_tgt,Eigen::Matrix4f &final_transform, bool downsample)
{
	//downsample=false;
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud_::Ptr src (new PointCloud_);
	PointCloud_::Ptr tgt (new PointCloud_);
	pcl::VoxelGrid<PointT_> grid;
	if (downsample)
	{
	grid.setLeafSize (0.01, 0.01, 0.01);
	grid.setInputCloud (cloud_src);
	grid.filter (*src);

	grid.setInputCloud (cloud_tgt);
	grid.filter (*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	pcl::NormalEstimation<PointT_, PointNormalT> norm_est;
	pcl::search::KdTree<PointT_>::Ptr tree (new pcl::search::KdTree<PointT_> ());
	norm_est.setSearchMethod (tree);
	//norm_est.setKSearch (50);
	norm_est.setKSearch (100);

	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	//reg.setTransformationEpsilon (1e-6);
	reg.setTransformationEpsilon (0.0001);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	//reg.setMaxCorrespondenceDistance (0.1);

	//reg.setMaxCorrespondenceDistance (0.0008);
	reg.setMaxCorrespondenceDistance (0.005);

	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

/*
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i)
	{
		//PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

			//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
*/

	reg.align (*reg_result);

	PCL_INFO("has converged with (%d) with score: (%f) \n", reg.hasConverged(), reg.getFitnessScore());
	Ti = reg.getFinalTransformation();

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


	//add the source to the transformed target
	*output += *cloud_tgt;

	final_transform = targetToSource;
 }



//Steal from https://github.com/ccny-ros-pkg/ccny_rgbd_tools/blob/master/ccny_rgbd/src/util.cpp
tf::Transform tfFromEigen(Eigen::Matrix4f t)
{
	tf::Transform tf;
	tf::Matrix3x3 m;
	m.setValue(t(0,0),t(0,1),t(0,2),
	t(1,0),t(1,1),t(1,2),
	t(2,0),t(2,1),t(2,2));
	tf.setBasis(m);
	tf.setOrigin(tf::Vector3(t(0,3),t(1,3),t(2,3)));
	return tf;
}




