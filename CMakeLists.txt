#
# Copyright (c) 2016 CNRS
# Authors: Florent Lamiraux
#
# This file is part of hpp-tom
# hpp-tom is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-tom is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-tom  If not, see
# <http://www.gnu.org/licenses/>.

cmake_minimum_required (VERSION 2.8)

include (cmake/base.cmake)
include (cmake/python.cmake)

set (PROJECT_NAME hpp-tom)
set (PROJECT_DESCRIPTION
  "hpp package for Technical University of Munich robot Tom"
  )
set (PROJECT_URL "")

setup_project ()

findpython ()

# install python files
install (FILES
  src/hpp/corbaserver/tom/robot.py
  src/hpp/corbaserver/tom/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/tom
  )
setup_project_finalize ()
