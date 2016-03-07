#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H

#include <fruit_rgbd.h>
#include <boost/make_shared.hpp>
#include <tf/transform_datatypes.h>
//convenient typedefs
typedef pcl::PointXYZ PointT_;
typedef pcl::PointCloud<PointT_> PointCloud_;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;





// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	MyPointRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};
//Steal from https://github.com/ccny-ros-pkg/ccny_rgbd_tools/blob/master/ccny_rgbd/src/util.h

tf::Transform tfFromEigen(Eigen::Matrix4f trans);
//void pairAlignICP (const PointCloud_::Ptr cloud_src, const PointCloud_::Ptr cloud_tgt,Eigen::Matrix4f &final_transform, bool downsample);

void pairAlignICP (const PointCloud_::Ptr cloud_src, const PointCloud_::Ptr cloud_tgt, PointCloud_::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);


void pairAlignNDT (const PointCloud_::Ptr input_cloud, const PointCloud_::Ptr target_cloud, PointCloud_::Ptr output_cloud,Eigen::Matrix4f &final_transform);



#endif // CLOUD_REGISTRATION_H

