# scenario_and_camera_based_navigation
ーーーーー編集中ーーーーー 

navigation software depend on camera image and scenario 


## Dependencies 
- ubuntu 20.04 LTS
- pythorch
- 
## Install 

### Step1
- PyTorch, CUDA-Toolkit 
```
pip3 install torch torchvision torchaudio scikit-image tensorboard
``` 

- mecab 
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

### step2
make workspace
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

## example 
### 1. learning (option)
```
cd catkin_ws/src/scenario_and_camera_based_navigation/experiments
```
#### path following module
```
./nav_cloning.sh
``` 
#### corridor clastify module 
```
./intersection_detector.sh
```
### 2. navigation
1. write a scenario `hoge/hoge`
2. move to the path and open three terminal
```
cd catkin_ws/src/scenario_and_camera_based_navigation/experiments/test
```
3. launch simulataion
```
./intersecton_detector.sh
./nav_cloning.sh
```
4. launch navigation
```
./scenario_navigation.sh
```
