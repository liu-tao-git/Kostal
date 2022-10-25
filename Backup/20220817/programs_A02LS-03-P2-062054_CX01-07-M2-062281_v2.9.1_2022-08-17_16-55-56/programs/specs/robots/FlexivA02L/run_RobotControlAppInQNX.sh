# Motion bar
devc-serusb -b115200
waitfor /dev/serusb1

# Robot Control App
cp RobotControlApp /tmp/
waitfor /tmp/RobotControlApp
on -C 3 /tmp/RobotControlApp ./specs/robots/FlexivA02L/ flexivCfg.xml
