# Plane Model

```c++
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), // pcd read
cloud_p (new pcl::PointCloud<pcl::PointXYZ>), // inlier data
cloud_f(new pcl::PointCloud<pcl::PointXYZ>); // outlier data, reuse for iteration
```

```c++
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

//Optional
seg.setOptimizeCoefficients(true);

//Methods
seg.setModelType (pcl::SACMODEL_PLANE); // SACMODEL_PLANE 사용
seg.setMethodType (pcl::SAC_RANSAC); // RANSAC 알고리즘 적용
seg.setMaxIterations (1000); // RANSAC iteration 횟수
seg.setDistanceThreshold (0.05); // 5cm
```

```c++
while(cloud->points.size() > 0.3 * nr_points){ // 전체 데이터의 30%까지 iteration
		std::cout << "number of point data left : " << cloud->points.size() << std::endl;
		seg.setInputCloud(cloud); 
		seg.segment (*inliers, *coefficients);
		
		//Extract the inlier
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

		//write
		pcl::PCDWriter writer;
		std::stringstream ss;
		ss << "inlier_plane_" << cloud->points.size() << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);

		//overwrite outlier to inliers
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud = *cloud_f;
	}

```

## Build

```bash
mkdir build 
cd build
cmake ..
make
```



## 실행

```bash
cd build
./pcd_read [.pcd file]
```

