# Dockers: ROS + Webots

Repo with everything necessary to run a docker image with ROS and webots, and demo a working pioneer3 mapping a "warehouse".

## Steps to run gmapping in the simulation:

1. Run `make webots_docker`. Go take a cup of coffee, this takes forever.
2. Run `./run` to start Docker. You'll need an NVIDIA graphics card.
3. First terminal:
  ```bash
  catkin_make
  roscore
  ```
4. In the second terminal run `webots src/pioneer_slam/worlds/warehouse.wbt`
5. Third terminal: `rosrun pioneer_slam pioneer3at`
6. Fourth terminal: 
```bash
rosrun gmapping slam_gmapping scan:=/pioneer3at/Sick_LMS_291/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2s
```
7. In the final terminal launch rviz to see the map visualization. You need to manually set up what you want to see in rviz.

End result should look something like this:

![screenshot](https://user-images.githubusercontent.com/10320441/87676447-021cb100-c74f-11ea-8055-2ad720aa03af.png)
