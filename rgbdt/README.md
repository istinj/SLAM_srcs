rgbdt: Simple RGB+Depth SLAM System
Author: Giorgio Grisetti

works on ROS Indigo + Ubuntu 14.04

### How do I get set up? ###

1) install the ubuntu packages

sudo apt-get install \
     libeigen3-dev \
     libflann-dev \
     libsuitesparse-metis-dev \
     freeglut3-dev \
     libqglviewer-dev 
     

2) checkout, install and build g2o. Locally.

cd <YOUR_SRC_FOLDER_WHERE_YOU_DOWNLOAD_EXTERNAL_LIBS>
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cmake ../
make -j <as much as you have>

g2o is now compiled, we need to set up the environment so that cmake finds our local installation.


To this end add this to your ~/.bashrc
export G2O_ROOT=<YOUR_SRC_FOLDER_WHERE_YOU_DOWNLOAD_EXTERNAL_LIBS>/g2o

source ~/.bashrc

IMPORTANT!
You might need to copy the g2o/build/g2o/config.h file in g2o/g2o

  g2o$ cp build/g2o/config.h g2o


3) download the thin drivers package (it's a ROS package) from
   https://github.com/grisetti/thin_drivers.git

   $> git clone https://github.com/grisetti/thin_drivers.git

   move *only* the thin_state_publisher folder to your catkin_workspace

   $> cd <your catkin workspace>/src
   $catkin_ws/src> ln -s <path_where_you_cloned_the_thin_drivers>/thin_state_publisher thin_state_publisher
   
   compile it with
  
   $> cd <your catkin workspace>/src
   $catkin_ws> catkin_make --pkg thin_state_publisher

4) move/link this package to your catkin workspace and build it
   $> cd <your catkin workspace>/src
   $> ln -s <path_where_you_cloned_rgbdt> rgbdt
   $catkin_ws> catkin_make --pkg rgbdt

That should do.


### How do I check if it works? ###

    download this file
    https://drive.google.com/file/d/0BzyJOKGGMQCxSm96elNudzF4V3c/view?usp=sharing
    

    uncompress it
    $> tar -xzvf mothers_house.txt.tgz

    run the tracker
    $> source catkin_ws/devel/setup.bash
    $> rosrun rgbdt lk_app mothers_house.txt  (from the folder where you uncompressed the tarball)

    two windows will pop up.
    press "c" on the image window to start the traking.
    There are other keys in the pool, have a look at the source (lk_app.cpp).

