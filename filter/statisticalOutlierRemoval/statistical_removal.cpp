#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main (int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// read cloud data, argv[1]
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> (argv[1], *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	// 최소 검색 이웃점 수
	sor.setMeanK (50);
	// mean + stddev_mult * sttdev, 최소 거리 
	sor.setStddevMulThresh (10.0);
	sor.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	// write to file
	// inliers
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("inliers_filtered.pcd", *cloud_filtered, false);
	
	//outliers
	sor.setNegative (true);
	sor.filter (*cloud_filtered);
	writer.write<pcl::PointXYZ> ("outliers_filtered.pcd", *cloud_filtered, false);

	return(0);
}
