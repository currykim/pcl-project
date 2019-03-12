#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	//Read
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> (argv[1], *cloud);
	
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
	ror.setInputCloud (cloud);
	ror.setRadiusSearch (0.1);
	ror.setMinNeighborsInRadius (1000);
	ror.setNegative (true);
	ror.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	//write
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("inliers.pcd", *cloud_filtered, false);

	return(0);
}
