Simulation Guide

## 1. install

### 1.1 ROS1 install

 This is a step-by--step guide to install and build all the prerquisties for running the  simulation on Ubuntu 22.04 with ROS noetic(include Gazebo 11)

 You might want to skip some steps if your system is already partially installed.

#### 添加ROS2 GPG密钥

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### 添加官方仓库

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 安装引导程序依赖项

主要是rosdep、vcstools等一系列用于源码安装的工具。  

```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool build-essential
```

初始化rosdep。  

```bash
sudo rosdep init
```

#### 安装hddtemp

```bash
cd ~/Downloads
wget http://archive.ubuntu.com/ubuntu/pool/universe/h/hddtemp/hddtemp_0.3-beta15-53_amd64.deb
sudo apt install ~/Downloads/hddtemp_0.3-beta15-53_amd64.deb
```

#### 修改rosdep初始化文件

下载base.yaml文件。  

```bash
d ~/Downloads
wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
```

打开base.yaml文件，Ubuntu22可以使用gedit修改。  

```bash
gedit ~/Downloads/base.yaml
```

搜索hddtemp，在ubuntu那一部分，添加上ubuntu22（最后一行，jammy那一行）的内容，如下所示：

```yaml
hddtemp:
  arch: [hddtemp]
  debian: [hddtemp]
  fedora: [hddtemp]
  freebsd: [python27]
  gentoo: [app-admin/hddtemp]
  macports: [python27]
  nixos: [hddtemp]
  openembedded: [hddtemp@meta-oe]
  opensuse: [hddtemp]
  rhel: [hddtemp]
  slackware: [hddtemp]
  ubuntu:
    '*': null
    bionic: [hddtemp]
    focal: [hddtemp]
    jammy: [hddtemp]
```

#### 修改20-default.list文件内容

修改`Rosdep init`创建的20-default.list文件内容，使其读取**本地的base.yaml文件**。  

```bash
sudo gedit /etc/ros/rosdep/sources.list.d/20-default.list
```

修改为如下内容：  

```yaml
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
# yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
# 确保下面的base文件路径和你的保存位置一致，前缀file://即为读取本地文件，/home/your_usernme/需要根据你的设置进行修改
# 也可以移动base.yaml文件到你需要的地方，保证下面的位置是你移动的位置即可
yaml file:///home/your_username/Downloads/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
```

#### 更新rosdep

```bash
rosdep update
```

可能会需要一定时间（需要连接到Github），可以考虑修改hosts文件，抑或其他帮助连接Github的方法，具体请自行查阅。  

#### 正式安装

#### 创建catkin工作区

主要用于存放安装Ros的源码工作区。  

```bash
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

安装Desktop full版本，更全面一点，代码如下：  

```bash
rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src
```

#### 解决依赖问题

安装Ros包可能有相互依赖的情况，如果缺少系统依赖则无法使用，需要使用rosdep工具自动检测并安装相应的[依赖文件](https://zhida.zhihu.com/search?content_id=241111139&content_type=Article&match_order=1&q=依赖文件&zhida_source=entity)

。  

```bash
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
```

#### 解决兼容问题

构建Ros1 Noetic之前，我们需要手动修补src文件夹中的两个包，以兼容Ubuntu22.04：

- rosconsole：Ros的控制台日志记录使用程序
- urdf：统一机器人描述格式（URDF）文件的解析器

相关的修补程序在Github上有开源实现，作者[Daniel Reuter](https://link.zhihu.com/?target=https%3A//github.com/dreuter)，参考链接：

- [GitHub — dreuter/rosconsole](https://link.zhihu.com/?target=https%3A//github.com/dreuter/rosconsole/tree/noetic-jammy)
- [GitHub — dreuter/urdf](https://link.zhihu.com/?target=https%3A//github.com/dreuter/urdf/tree/set-cxx-version)

使用Github上的修改包直接替换初始源码包。  

```bash
cd ~/ros_catkin_ws

# Download and use fix branch
git clone https://github.com/dreuter/rosconsole.git
cd rosconsole
git checkout noetic-jammy
cd ..
mv rosconsole src

git clone https://github.com/dreuter/urdf.git
cd urdf
git checkout set-cxx-version
cd ..
mv urdf src
```

#### 构建Ros1 Noetic

使用`catkin_make_isolated`命令构建Ros1 Noetic，并安装  

```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```

Ros1将会自动安装到所设置的catkin工作区中的install_isolated文件夹中，更新后即可正常使用Ros1了！  

```bash
source ~/ros_catkin_ws/install_isolated/setup.bash
```

可以直接将source命令写入bashrc中，避免每次使用时都需要更新。  

#### 其他

安装的Ros1 Desktop full版本并非完全包含所需要的Ros packages，可能还需要单独安装包。例如octomap、mavros等等。在Ubuntu20下安装方式为：  

```bash
sudo apt install ros-noetic-octomap ros-noetic-mavros
```

但从源码安装Ros1，相应的包也需要源码安装。过程如下。  

#### 生成包文件并自动下载依赖

```bash
cd ~/ros_catkin_ws
rosinstall_generator package1 package2 --rosdistro noetic --deps --tar > noetic-packages.rosinstall
vcs import --input noetic-packages.rosinstall ./src
```

package1和package2的位置替换为需要安装的包即可，注意不需要添加ros-noetic-的内容，也可以继续在后面放更多的包以一次性安装，但是个人建议还是单独安装比较稳妥(因为Github连接问题，vcs下载包的方式会导致必须所有依赖项都正确下载才能正常编译)。

哪怕第一次依赖A下载成功，依赖B失败，第二次依赖B成功，依赖A失败，两次结合起来，看似A和B都成功了，但实际上在第二次下载开始时，vcs工具会删除所有依赖。因此下载必须全部成功才行。

以 mavros 举例，安装方式为：  

```bash
cd ~/ros_catkin_ws
rosinstall_generator mavros --rosdistro noetic --deps --tar > noetic-packages.rosinstall
vcs import --input noetic-packages.rosinstall ./src
```

#### 替换rosconsole和urdf

新生成的包文件可能对 rosconsole 和 urdf 存在依赖，参考本文解决依赖问题部分进行替换即可。

#### 安装

```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
source ~/ros_catkin_ws/install_isolated/setup.bash
```



### 1.2  Autopilot install

To get the *very latest* (`demo_sitl` branch) version onto your computer, enter the following command into a terminal:

```
git clone https://github.com/ApexPilot2024/PX4_Firmware.git
cd PX4_Firmware
git checout demo_sitl
git submodule update --init --recursive
```

update gazebo and arm-gcc

```
./Tools/setup/ubuntu.sh
```

update xmlstarlet:

```
sudo apt install xmlstarlet
```

Now we  use Gazebo Classic on Ubuntu 22.04, so  you  can use the following commands to remove [Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/) (Harmonic) and then reinstall Gazebo-Classic 11:

```
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```

running the simulatuion:

```
cd /path/to/PX4_Firmware
make px4_sitl gazebo
```

If you run success, Now you can see :

![](/home/jason/catkin_ws/src/autonomous/docs/Screenshot from 2024-11-27 16-21-01.png)



### 1.3 Mavros install

Download and compile mavros:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:Autonomy2024/autonomous.git
cd autonomous
git submodule update --init --recursive
```

Install libgeographic and geographiclib

```
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

install geographiclib datasets:

```
cd ~/catkin_ws/src/autonomous/driver/mavros/mavros/scripts/
sudo ./install_geographiclib_datasets.sh
```



Because the order in which Gazebo looks for models is to search in .gazebo/models/ first, and then search in other paths, so when copying models to PX4 SITL, pay attention to whether there is a file with the same name under .gazebo/models/ (such as  stereo_camera) , if any, either delete the file with the same name, or replace the file with the same name.

#### 1.4 Update some gazebo models

Unzip models.zip to ~/.gazebo, Now you can see many gazebo models in ~/.gazebo/models/

Note:

You need to remove the stereo_camera 3d_lidar in  ~/.gazebo/models/

```
cd ~/.gazebo/models/
rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/
```



#### 1.6 VIO tools update

update nvidia-cuda:

```
sudo apt install nvidia-cuda-toolkit
```

Add these line to your ~/.bashrc and reload the terminal.

```
# cuda 10.2
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin
```

update ceres-solver:

```
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
mdkir build
cd build
cmake ..
make
sudo make install
```

#### 1.5  compile and running 

Add this to ~/.bashrc

```
gedit ~/.bashrc
# Add this source in .bashrc
source /YOUR_PATH/Tools/setup_gazebo.bash /YOUR_PATH/ /YOUR_PATH/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/YOUR_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/YOUR_PATH/Tools/sitl_gazebo

```

compile autonomous project:

```
cd ~/catkin_ws/
catkin_make or catkin build
```

Add this source in ~/.bashrc

```
gedit ~/.bashrc

source ~/catkin_ws/devel/setup.bash
```

run:

```
source ~/.bashrc
roslaunch px4 mavros_posix_sitl.launch
```

or run:

```
roslaunch px4 lidar_camera.launch
```

running VIO:

running flight control sim:

```
roslaunch px4 lidar_camera.launch
```

open a new terminal, running vio:

```
bash ~/catkin_ws/src/autonomous/scripts/drone_run_vio.sh
```

open a new terminal, transfer vio data to mavros and fcs:

```
cd ~/catkin_ws/src/autonomous/scripts
python vins_transfer.py iris 0
```

 or running RIO:

```
bash src/autonomous/scripts/run_rio.sh
```



#### 1.6 update ground station(QroundControl)

 Download QGroundControl.AppImage from this link: https://github.com/mavlink/qgroundcontrol/releases

and then give executable permissions

```
chmod +x  QGroundControl.AppImage
./QGroundControl,AppImage
```

NOTE: If you want to disable fuse gps, and only use vio to fuse , you can set ekf2_aid_mask  to 24. if you use RIO to fuse, you can set ekf2_aid_mask  to 256

