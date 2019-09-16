if [ $# -eq 1 ];then
    echo "build project!!!"
    if [ $1 == r ];then
	    echo "clean build file!!!!"
	    rm -rf build 
    fi
    bash build.sh all
fi
source build/install/setup.bash
mkdir -p build/install/cal_data
mkdir -p build/install/data/lidar
mkdir -p build/install/data/odom
#cyber_launch start modules/ee/launch/calib_a.launch
#cyber_launch start modules/ff/launch/calib_a.launch
#cyber_launch start modules/gg/launch/calib_a.launch
#cyber_launch start modules/calibration_8/launch/calib_a.launch

cyber_launch start modules/calibration_8_odometry/launch/calib_back.launch
cyber_launch start modules/calibration_8_odometry/launch/calib_left.launch
cyber_launch start modules/calibration_8_odometry/launch/calib_right.launch
cyber_launch start modules/calibration_8_odometry/launch/calib_check.launch
#cp build/install/cal_data/*.yaml ~/cybertron/params/test/

