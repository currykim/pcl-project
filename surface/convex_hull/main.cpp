#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>), cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);

	// PCD reader
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> (argv[2], *cloud);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	//Optional
	seg.setOptimizeCoefficients(true);

	//Methods
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.05);

	// Iteration
	int nr_points = (int) cloud->points.size();
	std::cout << "Input cloud data : " << nr_points << std::endl;

	while(cloud->points.size() > 0.3 * nr_points){

		std::cout << "number of point data left : " << cloud->points.size() << std::endl;
		seg.setInputCloud(cloud);
		seg.segment (*inliers, *coefficients);

		//Extract the planar
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

		// Use ConvexHull
		if (pcl::console::find_argument (argc, argv, "-cv") >= 0){
			pcl::ConvexHull<pcl::PointXYZ> chull;
			chull.setInputCloud(cloud_p);
			chull.reconstruct(*cloud_hull);

		// Use ConcaveHull
		}else if(pcl::console::find_argument (argc, argv, "-cc") >= 0){
			pcl::ConcaveHull<pcl::PointXYZ> chull;
			chull.setInputCloud(cloud_p);
			chull.setAlpha (0.1);
			chull.reconstruct(*cloud_hull);
		}

		//write
		pcl::PCDWriter writer;
		std::stringstream ss;
		ss << "inlier_plane_" << cloud_hull->points.size() << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str(), *cloud_hull, false);

		//overwrite outlier to inliers
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud = *cloud_f;
	}
	return(0);
}
