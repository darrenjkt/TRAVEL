# TRAVEL: Pointcloud Ground Segmentation
Original algorithm: https://github.com/url-kaist/TRAVEL

I wanted to use this ground segmentation in a python repository and so I converted the C++ functions to python apis. I use Open3D to interface with the function (see travel_demo.py for usage).

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

You might need to install: `apt install libpcl-dev`
