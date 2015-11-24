#! /usr/bin/env python
#| This file is a part of the ERC ResiBots project.
#| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
#| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
#|                      Antoine Cully, cully@isir.upmc.fr
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.

import limbo
import ode
import os
def options(opt):
    opt.add_option('--robdyn', type='string', help='path to robdyn', dest='robdyn')
    opt.add_option('--robot', type='string', help='build also for real robot[true/false]', dest='robot')

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

    if bld.options.robot and bld.options.robot == 'true':
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

    if bld.options.robot and bld.options.robot == 'true':
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
