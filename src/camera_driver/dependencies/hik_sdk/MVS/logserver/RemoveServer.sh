#!/bin/sh
DIRNAME=`dirname $0`
PWD=`pwd`

PIDFILE="/var/run/MvLogServer.pid"

USER_ID=`id -u`
# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "LogServer can only be Removed by root user or sudoer"
exit 1
fi

# ARCH=$(uname -m | sed 's/x86_//;s/i[3-6]86/32/')

if [ -f /etc/debian_version ]; then
    OS=Debian  # XXX or Ubuntu??
    VER=$(cat /etc/debian_version)
elif [ -f /etc/redhat-release ]; then
    # TODO add code for Red Hat and CentOS here
    OS=Redhat  # XXX or CentOs??
    VER=$(uname -r)
else
    OS=$(uname -s)
    VER=$(uname -r)
fi

service MvLogServer stop  2> /dev/null

if [ "Debian" = "$OS" ]; then
	update-rc.d -f MvLogServer remove 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
	chkconfig MvLogServer off
    chkconfig --del MvLogServer
else
	echo "Can not recognize system"
fi

rm -f /etc/init.d/MvLogServer

rm -f $PIDFILE

