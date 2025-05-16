# scenario_and_camera_based_navigation

Navigation software based on camera images and scenarios

<!-- <img src="system.png"> -->

## Dependencies 
- ubuntu 20.04 LTS
- ROS Noetic
- PyTorch

## Install 

### Step 0: Install ROS Noetic and NVIDIA drivers

### Step 1: Install required Python libraries and MeCab
- PyTorch, CUDA-Toolkit 
```
pip3 install torch torchvision torchaudio torcheval scikit-image tensorboard 
``` 

- MeCab 
``` 
sudo apt install mecab -y 
sudo apt install libmecab-dev
pip3 install mecab-python3
git clone --depth 1 https://github.com/neologd/mecab-ipadic-neologd.git
cd mecab-ipadic-neologd
./bin/install-mecab-ipadic-neologd -n
yes
sudo apt install -y ros-noetic-jsk-visualization
sudo cp /etc/mecabrc /usr/local/etc/
``` 

### Step 2: Build the Workspace
```
cd
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src/
git clone https://github.com/open-rdc/scenario_and_camera_based_navigation.git
wstool init
wstool merge scenario_and_camera_based_navigation/package.install
wstool up
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
./real_tsudanuma2-3_sim/setup/step1.bash
source ~/.bashrc
cd ../
sudo apt install python3-catkin-tools -y
catkin build
source ~/.bashrc
source devel/setup.bash
```

## Example Usage
### 1. Learning (option)
```
cd catkin_ws/src/scenario_and_camera_based_navigation/experiments
```
-  path following module
```
./nav_cloning.sh
``` 
- corridor clastify module 
```
./intersection_detector.sh
```
### 2. Navigation
1. Write a scenario file(optional) 

   Create and edit a scenario file in the following directory:

    `scenario_navigation/config/Scenario/hoge.txt`

2. Set the scenario_path 


   Edit the launch file `scenarion_navigation/launch/scenario_navigation.launch`
    to specify which scenario file to use:

```scenarion_navigation/launch/scenario_navigation.launch 
<launch>
  <node name="cmd_dir_executor" pkg="scenario_navigation" type="cmd_dir_executor_detailed" output="screen" />
  <node name="scenario_parser" pkg="scenario_navigation" type="scenario_parser_jsk.py" output="screen" >
    <param name="scenario_path" value="$(find scenario_navigation)/config/Scenarios/scenario01.txt" />
    # if you write scenario, change under value to your file name
    <param name="scenario_path" value="$(find scenario_navigation)/config/Scenarios/hoge.txt" />
  </node>
</launch>
```
3. Move to the test directory and open three terminals: 
```
cd ~/catkin_ws/src/scenario_and_camera_based_navigation/experiments/test
```
4. Launch the simulataion
```
./intersecton_detector.sh
./nav_cloning.sh
```
5. Launch scenario-based navigation
```
./scenario_navigation.sh
```
