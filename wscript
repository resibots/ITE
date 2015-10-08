#! /usr/bin/env python
import limbo
import ode
import os
def options(opt):
    print "nothing"

def build(bld):
    print bld

    libs = 'ODE EIGEN ROBDYN HEXACONTROL BOOST BOOST_TIMER ODE BOOST_SYSTEM BOOST_THREAD BOOST_SERIALIZATION BOOST_FILESYSTEM ROS DYNAMIXEL IMU_RAZOR ROS LIMBO SFERES2 BOOST_CHRONO RT'
    cxxflags = bld.get_env()['CXXFLAGS']

    bld.get_env()['INCLUDES_ROBDYN']= os.environ['RESIBOTS_DIR'] + '/include/robdyn'
    bld.get_env()['LIBPATH_ROBDYN']= os.environ['RESIBOTS_DIR'] + '/lib/'
    bld.get_env()['LIB_ROBDYN']=['robdyn','robdyn_osgvisitor']

    bld.get_env()['INCLUDES_ROS']='/opt/ros/' + os.environ['ROS_DISTRO'] + '/include'
    bld.get_env()['LIBPATH_ROS']='/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/'
    bld.get_env()['LIB_ROS']=['roscpp','rosconsole','roscpp_serialization','rostime','pthread','cpp_common', 'xmlrpcpp','rosconsole_log4cxx', 'rosconsole_backend_interface'] # 'xmlrpc','std_msgs'

    bld.get_env()['INCLUDES_HEXACONTROL']= os.environ['HOME'] + '/catkin_ws/devel/include/'

    libs += ' OSG'
    bld.get_env()['LIB_OSG'] = ['osg','osgGA', 'osgDB', 'osgUtil',
                           'osgViewer', 'OpenThreads',
                           'osgFX', 'osgShadow']


    ode.check_ode(bld)


    obj = bld(features = 'cxx cxxstlib',
              source = 'simu.cpp controllerDuty.cpp hexapod.cc',
              includes = '. .. ../../',
              target = 'hexapod',
              uselib =  libs,
              use = 'limbo')

    obj = bld(features = 'cxx cxxstlib',
              source = 'simu.cpp controllerDuty.cpp hexapod.cc',
              includes = '. .. ../../',
              target = 'hexapod_graphic',
              uselib =  libs,
              cxxflags = ['-DGRAPHIC'],
              use = 'limbo')


#    bld(features='cxx cxxstlib', source='simu.cpp hexapod.cc controllerDuty.cpp', target='hexapod',uselib=libs, uselib_local='limbo')


    limbo.create_variants(bld,
                          source = 'hexa_bomean.cpp',
                          uselib_local = 'limbo hexapod',
                          uselib = libs,
                          includes=". ../../src ../ ",
                          variants = ['ROBOT'])

    limbo.create_variants(bld,
                          source = 'hexa_bomean.cpp',
                          uselib_local = 'limbo hexapod_graphic',
                          uselib = libs,
                          includes=". ../../src ../ ",
                          variants = ['GRAPHIC'])
