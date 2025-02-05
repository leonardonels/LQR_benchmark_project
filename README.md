<div align="center">
    <h1>LQR_project</h1>
</div>

## :open_file_folder: What's in this repo

* cpp folder for algorithms
* build folder for compiled algorithms
* nanoflann fold in which **should** be cloned the nanoflan repo
* odometry folder for odometry csv files
* trajectories folder for trajectories csv files
* closest_point.csv file in which the closest point found is saved for future plotting
* benchmark.py file to use to make algorithms testing easier
* utils folder for additional files and functions like
    * print.py file used to plot the closest point found alongside the trajectory used and the odometry 
    * time.csv a simple but effective file used to save the execution time

## :package: Prerequisite packages
> What we need are pandas and numpy.

```commandline
sudo apt-get install python3-numpy python3-pandas -y
```
## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leonardonels/LQR_benchmark_project.git
cd ~/ros2_ws/src/LQR_benchmark_project
git submodule update --init
g++ -o build/knn cpp/kd_tree_nearest_point.cpp
```
