#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/src/ros_numpy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/install/lib/python2.7/dist-packages:/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/build" \
    "/usr/bin/python" \
    "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/src/ros_numpy/setup.py" \
    build --build-base "/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/build/ros_numpy" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/install" --install-scripts="/home/cc/ee106a/fa19/class/ee106a-abs/ros_workspaces/project/install/bin"
