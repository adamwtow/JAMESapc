#include "cloud_filtering_tools.h"


PointCloud::Ptr filter_cloud(PointCloud::Ptr cloud){

    PointCloud::Ptr cloud_filtered (new PointCloud);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50); //50
    sor.setStddevMulThresh (1.0);
    sor.filter(*cloud_filtered);

    //std::cerr << "Cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;

}


/*pcl::PointCloud<pcl::PointNormal>::Ptr smooth_cloud(PointCloud::Ptr cloud){
    // Load input file into a PointCloud<T> with an appropriate type

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    //mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    mls.process (*mls_points);
}*/


pcl::PointCloud<pcl::Normal>::Ptr compute_normals(PointCloud::Ptr cloud){

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;


    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
