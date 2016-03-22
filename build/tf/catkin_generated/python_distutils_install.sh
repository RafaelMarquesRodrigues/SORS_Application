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

echo_and_run cd "/home/rafael/SORS_Application/src/tf"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/rafael/SORS_Application/install/lib/python2.7/dist-packages:/home/rafael/SORS_Application/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rafael/SORS_Application/build" \
    "/usr/bin/python" \
    "/home/rafael/SORS_Application/src/tf/setup.py" \
    build --build-base "/home/rafael/SORS_Application/build/tf" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/rafael/SORS_Application/install" --install-scripts="/home/rafael/SORS_Application/install/bin"
