#=================
# System update
#=================
if [ -f /root/mnt/system_update/.update_programs.sh ]; then
    /bin/sh /root/mnt/system_update/.update_programs.sh
    exit 0
fi

#=================
# Clean up outdated logs
#=================
# keep csv files in 3 days
find . -name "*.csv" -mtime +3 -remove!
# keep log files in 7 days
find . -name "*.log" -mtime +7 -remove!

#=================
# Run UpdateService
#=================
if [ -f UpdateService ]; then
	reopen /dev/con3
	cp UpdateService /tmp/
	on -C 1 nice -n-10 /tmp/UpdateService ./specs/robots/FlexivA02L/ &
fi

#=================
# Run firmware update helper
# Upon finish, clear contents in update helper and exit script
#=================
./firmware_update/update_helper.sh

if [ $? == 1 ] ; then
	>./firmware_update/update_helper.sh
	exit 1
fi

#=================
# RobotControlApp
#=================
dumper -d /root/mnt/tmp/ &

#=================
# RobotControlApp
#=================
sleep 1
reopen /dev/con2
devc-serusb -b115200
waitfor /dev/serusb1

cp RobotControlApp /tmp/
# Launch RCA without CLI (with noCli option)
on -C 3 /tmp/RobotControlApp ./specs/robots/FlexivA02L/ flexivCfg.xml noCli &

#=================
# ArmDriver
#=================
sleep 10
reopen /dev/con1
cp ArmDriver /tmp/
on -C 0 /tmp/ArmDriver -i8254x 1 1 -t 0 -dcmmode busshift -v 1 -eni eni_bub_bcb_abb_7x_wsb.xml -cfg A02LS-03-P2-062054/arm_driver_param.xml -sn A02LS-03-P2-062054 -lcns CX01-07-M2-062281.lic &

