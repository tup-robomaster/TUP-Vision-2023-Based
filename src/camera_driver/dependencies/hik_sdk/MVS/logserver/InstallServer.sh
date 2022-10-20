#!/bin/bash
DIRNAME=`dirname $0`
PWD=`pwd`
LOGSERVER_PATH=$(cd `dirname $0`; pwd)
SDK_HOME=$(cd ${LOGSERVER_PATH}/../; pwd)
USER_ID=`id -u`
SDK_PATH_OLD=$(cat ${LOGSERVER_PATH}/Debian/MvLogServerd | grep "^SDK_HOME=" | awk -F '=' '{print $2}')
FIND_PATH=${SDK_PATH_OLD//\//\\\/}
FIND_PATH=${FIND_PATH//\#/\\\#}
REPLACE_PATH=${SDK_HOME//\//\\\/}
# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "LogServer can only be installed by root user or sudoer"
exit 1
fi
  
ARCH=$(uname -m | sed 's/x86_//;s/i[3-6]86/32/')

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

if [ "Debian" = "$OS" ]; then
	sed -i "/^SDK_HOME=*/s/${FIND_PATH}/${REPLACE_PATH}/" ${LOGSERVER_PATH}/Debian/MvLogServerd
	cp -f ${LOGSERVER_PATH}/Debian/MvLogServerd /etc/init.d/
elif [ "Redhat" = "$OS" ]; then
	sed -i "/^SDK_HOME=*/s/${FIND_PATH}/${REPLACE_PATH}/" ${LOGSERVER_PATH}/Redhat/MvLogServerd
	cp -f ${LOGSERVER_PATH}/Redhat/MvLogServerd /etc/init.d/
else
	sed -i "/^SDK_HOME=*/s/${FIND_PATH}/${REPLACE_PATH}/" ${LOGSERVER_PATH}/Debian/MvLogServerd
	cp -f ${LOGSERVER_PATH}/Debian/MvLogServerd /etc/init.d/
fi

mv -f /etc/init.d/MvLogServerd /etc/init.d/MvLogServer
chmod 777 /etc/init.d/MvLogServer


if [ "Debian" = "$OS" ]; then
	update-rc.d -f MvLogServer defaults 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
	chkconfig --add MvLogServer
	chkconfig MvLogServer on
else
	echo "Can not recognize system"
fi

service MvLogServer start