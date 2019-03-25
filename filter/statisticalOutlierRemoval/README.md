# Statistical Outlier Removal

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // pcd read
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); // save inlier data
```

```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (cloud); // pcd 파일 읽기
sor.setMeaK (50); //최소 이웃점 객수
sor.setStddevMulThresh (1.0); // mean + stddev_mult * stddev, 최소 거리
sor.filter(*cloud_filtered); // inlier 데이터 저장
```


## 실행

```bash
cd build
./statistical_removal [.pcd file]
```
참조 : http://pointclouds.org/documentation/tutorials/statistical_outlier.php
