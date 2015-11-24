#! /usr/bin/env python
import limbo
import ode
import os
def options(opt):
    opt.add_option('--robdyn', type='string', help='path to robdyn', dest='robdyn')
    opt.add_option('--robot', type='string', help='build also for real robot', dest='robot')

def build(bld):
    libs = 'ODE EIGEN ROBDYN BOOST BOOST_TIMER BOOST_SYSTEM BOOST_THREAD BOOST_SERIALIZATION BOOST_FILESYSTEM DYNAMIXEL LIMBO SFERES2 BOOST_CHRONO RT'
    cxxflags = bld.get_env()['CXXFLAGS']

    if bld.options.robdyn:
      robdyn_includes = bld.options.robdyn + '/include/robdyn'
      robdyn_libs = bld.options.robdyn + '/lib'
    elif 'RESIBOTS_DIR' in os.environ:
      robdyn_includes = os.environ['RESIBOTS_DIR'] + '/include/robdyn'
      robdyn_libs = os.environ['RESIBOTS_DIR'] + '/lib/'
    else:
      robdyn_includes = ['/usr/include/robdyn', '/usr/local/include/robdyn']
      robdyn_libs = ['/usr/lib', '/usr/local/lib']

    bld.get_env()['INCLUDES_ROBDYN']= robdyn_includes
    bld.get_env()['LIBPATH_ROBDYN']= robdyn_libs
    bld.get_env()['LIB_ROBDYN']=['robdyn','robdyn_osgvisitor']

    if bld.options.robot:
      bld.get_env()['INCLUDES_ROS']= '/opt/ros/' + os.environ['ROS_DISTRO'] + '/include'
      bld.get_env()['LIBPATH_ROS']= '/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/'
      bld.get_env()['LIB_ROS']= ['roscpp','rosconsole','roscpp_serialization','rostime','pthread','cpp_common', 'xmlrpcpp','rosconsole_log4cxx', 'rosconsole_backend_interface'] # 'xmlrpc','std_msgs'

      bld.get_env()['INCLUDES_HEXACONTROL']= os.environ['HOME'] + '/catkin_ws/devel/include/'

      libs += ' HEXACONTROL ROS'

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

    if bld.options.robot:
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
