# scenario_navigation  

### Requirement  

Mecab
```
sudo apt install mecab
sudo apt install libmecab-dev
pip3 install mecab-python3
```
```
git clone --depth 1 https://github.com/neologd/mecab-ipadic-neologd.git
cd mecab-ipadic-neologd
./bin/install-mecab-ipadic-neologd -n
```
Install this package and msgs
```
git clone https://github.com/haruyama8940/scenario_navigation_msgs.git
git clone https://github.com/haruyama8940/scenario_navigation.git
```
JSK　plugin
```
sudo apt install -y ros-noetic-jsk-visualization
```
### Execute  

- Simulator (without vison)  
1) Launch the simulator  


2) Move robot to initial position  

3) Select a scenario in "navigation_cmd.launch"  
`<param name="scenario_path" value="$(find scenario_navigation)/config/Scenarios/scenario01.txt" />`

4) Execute the scenario navigation  with target direction
`roslaunch scenario_navigation navigation_cmd.launch`


- Real Robot (with vison)  
1) Execute the scenario navigation
`roslaunch scenario_navigation navigation_cmd.launch`  

### Trouble-shooting
- Mecab errors
```
error message: [ifs] no such file or directory: /usr/local/etc/mecabrc
```
Please check the installation path of mecabrc & copy
```
 sudo cp /etc/mecabrc /usr/local/etc/
```
