# Convex Hull & Concave Hull

```c++
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
cloud_p (new pcl::PointCloud<pcl::PointXYZ>), 
cloud_f(new pcl::PointCloud<pcl::PointXYZ>), 
cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
```

```C++
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
```

`-cv ` : Convex Hull

`-cc` : Concave Hull



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
./pcd_read -cv [.pcd file] ## for Convex Hull
./pcd_read -cc [.pcd file] ## for Concave Hull
```



## 결과

```bash
ls *.pcd
inlier_pcd_[inlier 데이터 갯수1].pcd
inlier_pcd_[inlier 데이터 갯수2].pcd
```



참조 : 

http://pointclouds.org/documentation/tutorials/extract_indices.php

http://pointclouds.org/documentation/tutorials/convex_hull_2d.php
