# TRAVEL: Pointcloud Ground Segmentation
Original algorithm: https://github.com/url-kaist/TRAVEL

I wanted to use this ground segmentation in a python repository and so I converted the C++ functions to python apis. I use Open3D to interface with the function (see travel_demo.py for usage). 

Note that this only does the ground segmentation part of TRAVEL, not the object clustering.

Additional changes: Modified the estimateGround function to return ground_inds, rather than the separated ground and obj pcd to allow for indexing the original input pointcloud and retaining extra channel information.

# Pre-Requisites
Apt libraries:
```
apt update && apt install libeigen3-dev libflann-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libboost1.65-dev
```
Download BOOST lib (https://www.boost.org/doc/libs/1_65_1/more/getting_started/unix-variants.html)
```
wget https://boostorg.jfrog.io/artifactory/main/release/1.65.1/source/boost_1_65_1.tar.gz
tar -xvf boost_1_65_1.tar.gz
cd boost_1_65_1 && ./bootstrap.sh # Downloads boost to /usr/local
./b2 install
```  
Download pcl-1.8 library (https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)
```
wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.0.tar.gz
tar -xvf pcl-1.8.0.tar.gz
cd pcl-1.8.0 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
make -j2 install
```
Change the path of pcl-1.8 based on where it installs to. Default is: "/usr/local/include/pcl-1.8/"

To include in any project, structure the files like this:
```
my_project
- travel
  - travel_api.py 
  - src
    - tgs.hpp
    - travel_pybind.cpp
  
```
Then in the setup.py file, 
```
ext_modules=[
        Pybind11Extension(
            name='travel.ground_seg', 
            sources=['travel/src/travel_pybind.cpp'],
            include_dirs=[
            os.environ.get("EIGEN_INCLUDE_DIR", "/usr/include/eigen3/"),
            "/usr/local/include/pcl-1.8/"],          
            extra_compile_args=['-std=c++14'],
            ), 
            ...OtherExtensions...]
```

### Troubleshooting
If the above doesn't work, a dirty workaround is to keep this repo's setup.py in the travel folder and build it from there.
```
my_project
- travel
  - travel_api.py 
  - setup.py
  - src
    - tgs.hpp
    - travel_pybind.cpp
```    
Building with either `python setup.py develop` or `pip install -e . --user` should work.

# How to run
Run docker
```
bash docker/run.sh
```

### Running TRAVEL in python
```
python setup.py develop
python travel_demo.py
```


### Running TRAVEL in cpp (without ROS)
Build files
```
mkdir build && cd build
cmake ..
make
```
Run the program with 
```
./travel
```
