#!/bin/sh

################################################################################
#
# unload.sh
# for gevfilter.ko unload
#
################################################################################

# Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    unload.sh - Unload For Ethernet driver module "
    echo "                gevfilter.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash unload.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Unload the eBUS Universal Pro For Ethernet module and remove the configure"
	echo "    from the system to be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

# Print out the error and exit 
#  $1 Error message
#  $2 Exit code
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

# Ensure the module is in memory
IMAGEFILTER_LOADED=`lsmod | grep -o gevfilter`
if [ "$IMAGEFILTER_LOADED" != "gevfilter" ];then
	echo "already have no gevfilter"
	exit 0
fi

# Unload the module
echo "Unloading GigEVision Image Filter For Ethernet..."
IMAGEFILTER_LOADED=`lsmod | grep gevfilter | cut -d ' ' -f1`
/sbin/rmmod $IMAGEFILTER_LOADED $* || exit 1

