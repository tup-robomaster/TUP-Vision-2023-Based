#!/bin/sh

################################################################################
#
# load.sh
# for gevfilter.ko load
#
################################################################################

# Variables
DIRNAME=$(cd `dirname $0`; pwd)
DRIVER_HOME=${DIRNAME}
HOST_VERSION=`uname -r`
HOST_ARCH=`uname -m | sed -e 's/i.86/i686/' -e 's/^armv.*/arm/'`

#Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    load.sh - Load For Ethernet driver module "
    echo "              gevfilter.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash load.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Load For Ethernet module and configure the system to"
    echo "    be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

#Print out the error and exit 
# $1 Error message
# $2 Exit code
ErrorOut()
{
	echo ""
	echo "Error: $1"
	echo ""
	exit $2
}

# Parse the input arguments
for i in $*
do
    case $i in        
        --help)
            DisplayHelp
            exit 0
        ;;
        *)
        # unknown option
        DisplayHelp
        exit 1
        ;;
    esac
done

# Check required priviledge
if [ `whoami` != root ]; then
	ErrorOut "This script can only be run by root user or sudoer" 1
fi

# Do not re-load if not needed
IMAGEFILTER_LOADED=`lsmod | grep -o gevfilter`
if [ "$IMAGEFILTER_LOADED" = "gevfilter" ];then
	echo "already have gevfilter"
	exit 0
fi

# Sanity check
if [ "`uname -m`" = "x86_64" ]\
   || [ "`uname -m`" = "i386" ]\
   || [ "`uname -m`" = "aarch64" ]\
   || [ "`uname -m`" = "armv7l" ]\
   || [ "`uname -m`" = "i686" ]
then
	echo "The arch is: `uname -m`"
else
	ErrorOut "Driver Not Support the arch `uname -m` " 1
fi

if [ ! -f $DRIVER_HOME/gevfilter.ko ]; then
	ErrorOut "gevfilter.ko isn't build yet " 1
fi

# Load the module
echo "Loading GigEVision Image Filter For Ethernet for `uname -m` ..."

/sbin/insmod $DRIVER_HOME/gevfilter.ko

sleep 1

chmod 777 /dev/HikGEVFilter

echo "end..."

