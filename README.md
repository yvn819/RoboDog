# RoboDog

Here is a short introduction to our RoboDog project. The main task of our RoboDog is to avoid obstacles by sensing its surroundings with the depth camera.
## Getting started

1. Clone the repository and `cd` to the `i2ros_team28` folder.  

2. Before building, you may need to install some ros packages. In one terminal, run:

```
sudo apt-get install ros-noetic-octomap 
sudo apt-get install ros-noetic-octomap-server 
sudo apt-get install ros-noetic-octomap-ros
sudo apt-get install ros-noetic-octomap-rviz-plugins
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-map-server 
sudo apt-get install ros-noetic-global-planner
```

3. Then build the project, run

```
catkin build
```

4. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/

5. Unzip the Unity file and copy the files to `your_repo_path/devel/lib/simulation/`

6. You'll need 2 terminals to launch all the nodes. Remember to source the environment when you launch a new terminal. 
    ```
    source devel/setup.bash
    ```

    - In first terminal, execute  

    ```
    roslaunch simulation simulation.launch
    ```

    - In second terminal, execute 

    ```
    roslaunch controller_pkg controller.launch
    ```


## Tips

We also provide a perceptual approach using dynamically published maps, see branch `RoboDogBackup`

- First switch to the branch `RoboDogBackup`

```
git checkout RoboDogBackup
```
- Repeat steps 3-6 in `getting started` section.





## More Info

Check out `Documentation` folder to get more information about RoboDog. Such as demo video, RQT graph. More details about the code can be found in the `Final Report-team 28` in the same folder!

<!-- ## Authors

Contributors names and contact info.

- Lin Run
    [@runlin]

- Zhang Yihan
    [@00000000014AD4EA]

- Xin Yutong
    [@00000000014ABFEA] -->






