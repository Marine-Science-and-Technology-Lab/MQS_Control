# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmocap_base_driver;-lmocap_kalman_filter".split(';') if "-lmocap_base_driver;-lmocap_kalman_filter" != "" else []
PROJECT_NAME = "mocap_base"
PROJECT_SPACE_DIR = "/home/lab/mqs/install"
PROJECT_VERSION = "0.0.1"
