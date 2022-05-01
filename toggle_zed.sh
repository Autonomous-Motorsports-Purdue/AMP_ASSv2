# Get directory of the shell script
BASEDIR=$(dirname $0)

if [ -f ${BASEDIR}/src/zed-ros-wrapper/CATKIN_IGNORE ]; then
    rm ${BASEDIR}/src/zed-ros-wrapper/CATKIN_IGNORE
else
    touch ${BASEDIR}/src/zed-ros-wrapper/CATKIN_IGNORE
fi
