#!/bin/sh

# $Id: in-screen.sh 15909 2013-01-26 21:21:31Z jemian $

/usr/bin/screen -dm -S ioc9idm1k -h 100000 ./run

# start the IOC in a screen session
#  type:
#   screen -r   to start interacting with the IOC command line
#   ^a-d        to stop interacting with the IOC command line
#   ^c          to stop the IOC
