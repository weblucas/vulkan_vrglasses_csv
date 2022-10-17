# vrglasses_csv

Ubuntu 18.04

select a INSTALL_FOLDER for this project

## installation:
```
apt install libeigen3-dev libvulkan-dev libgoogle-glog-dev libglm-dev glslang-tools libopencv-dev
cd INSTALL_FOLDER
git clone git@github.com:weblucas/vulkan_vrglasses_csv.git

cd INSTALL_FOLDER/vrglasses_for_robots/shaders/
sh build_vrglasses4robots_shaders.sh

cd INSTALL_FOLDER/vrglasses_for_robots
mkdir build
cd INSTALL_FOLDER/vrglasses_for_robots/build
cmake ..
make -j8
```
## Download data 
download the content of the link https://drive.google.com/drive/folders/18S7sxqXU2LIjCSeEzLRQOXcqI3ZK2jBQ?usp=sharing
to your DATA_FOLDER
adjust the file DATA_FOLDER/vk_glasses_csv_flags.txt with the correct filepaths
## Running 
```
/content/workspace/vrglasses_for_robots/build/vrglasses4robots_csv --flagfile=DATA_FOLDER/vk_glasses_csv_flags.txt
```
