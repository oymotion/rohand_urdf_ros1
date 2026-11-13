# ROHAND URDF PACKAGE FOR ROS1

rohand urdf package

## 1.Clone

```BASH
cd ~
mkdir -p ros1_ws/src
cd ros1_ws/src
git clone ssh://git@github.com/oymotion/rohand_urdf_ros1
```

## 2.Install ros-noetic-urdfdom-py

```BASH
sudo apt update
sudo apt install ros-noetic-urdfdom-py
```

## 3.Compile

```BASH
chmod +x rohand_urdf_ros1/scripts/rohand_joint_state_gui.py
chmod +x rohand_urdf_ros1/scripts/rohand_urdf.py
cd ~
cd ros1_ws/
catkin_make
source devel/setup.bash
```

## 4.Node rohand_urdf

| Topic             | Description                                                               |
| ----------------- | ------------------------------------------------------------------------- |
| "if_slider_link" | slider for index finger, control index finger by changing it's position   |
| "mf_slider_link" | slider for middle finger, control middle finger by changing it's position |
| "rf_slider_link" | slider for ring finger, control ring finger by changing it's position     |
| "lf_slider_link" | slider for little finger, control little finger by changing it's position |
| "th_slider_link" | slider for thumb, control thumb by changing it's position                 |
| "th_root_link"   | slider for thumb root, control thumb root by changing it's position       |

## 5.RUN

Launch 'display_r.py or display_l.py' file:

Left hand：

```BASH
roslaunch rohand_urdf_ros1 display_l.launch
```

Right hand：

```BASH
roslaunch rohand_urdf_ros1 display_r.launch
```

## 6.TIPS

If you want to permanently close the warning pop-up window(when it pops up):

```BASH
echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc
source ~/.bashrc
```

