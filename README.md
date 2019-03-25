# Point Cloud Library (PCL) & Cloud Compare 설치 및 실행

설치 환경 :

`Ubuntu 16.04.6 LTS`

`g++ , 5.4.0`



# 1. Dependency 설치

PCL 설치에 앞서 dependency 소프트웨어 설치를 진행합니다.



### 1.1 eigen3 설치

eigen 라이브러리는 C++에서 동작하는 매트릭스 연산 및 선형연산을 지원합니다. [링크](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

`apt-get`명령을 통해 설치를 진행합니다

```bash
sudo apt-get install libeigen3-dev
```



### 1.2 FLANN 설치 

flann은 C, C++, Python등의 언어에서 지원하는 2D/3D 다차원 Nearest Neighbor search를 제공합니다. [링크](https://www.cs.ubc.ca/research/flann/)

소스를 받아 빌드하여 설치합니다.

```bash
sudo apt-get install wget
wget https://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
unzip flann-1.8.4-src.zip && cd flann-1.8.4-src
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```



### 1.3 VTK 설치

VTK == Visualization TooKit 으로 PCL의 시각화 용도로 사용됩니다. 소스를 받아 빌드하여 설치합니다.

```bash
wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
tar xvf VTK-7.1.0.tar.gz && cd VTK-7.1.0
mkdir build && cd build 
cmake ..
make -j$(nproc)
sudo make install
```
** openGL 에러있을 시
```bash
sudo apt-get install build-essential
sudo apt-get install freeglut3-dev libglu1-mesa-dev mesa-common-dev
```


### 1.4 Boost 설치

C++ 프로그래밍 언어를 위한 선형대수, 영상 처리와 같은 작업들과 구조들을 지원하는 비표준 라이브러리들의 집합입니다.

`apt-get`명령을 통해 설치합니다.

```bash
sudo apt-get install libboost1.58-all-dev
```





## 2. Point Cloud Library(PCL) 설치

`snap` 명령을 통해 설치하거나 소스코드를 다운받아 빌드하여 설치합니다.

```bash
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar xvf pcl-1.8.1.tar.gz && cd pcl-pcl-1.8.1
mkdir build && cd build
cmake .. (or cmake-gui ..)
make -j$(nproc)
```

### 2.1 Validation

다음은 설치확인을 위한 과정입니다. 

`test.cpp` 파일을 생성

```cpp
#include <iostream>
int main(){
    std::cout << "Hello, World!" << std::endl;
    return(0);
}
```

CMakelists.txt 생성

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-test)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl-test test.cpp)
target_link_libraries(pcl-test ${PCL_LIBRARIES})

SET(COMPILE_FLAGS "-std=c++11")
add_definitions(${COMPILE_FLAGS})
```

빌드

```bash
mkdir build && cd build
cmake ..
make
```

```bash
./pcl_test
```

Output으로 `Hello, World!`가 출력되는지 확인합니다.



## 3. Cloud Compare 설치 & 실행

간단히 `snap`명령을 통해 설치가 가능합니다.

```bash
snap install cloudcompare
```

쉘상에서 명령어로 실행이 가능합니다.

CloudCompare :

`$ cloudcompare.CloudCompare`

ccViewer (PCD 뷰어 전용):

`$ cloudcompare.ccViewer [.pcd 파일]`



`** PCD 불러오기 실패 시`

```bash
sudo snap refresh --edge cloudcompare
```



