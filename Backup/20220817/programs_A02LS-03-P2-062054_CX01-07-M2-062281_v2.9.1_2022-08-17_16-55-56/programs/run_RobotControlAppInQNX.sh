# slay app first
slay RobotControlApp

# Motion bar
slay -f devc-serusb
devc-serusb -b115200
waitfor /dev/serusb1

# Robot Control App
cp RobotControlApp /tmp/
on -C 3 /tmp/RobotControlApp ./specs/robots/FlexivA02L/ flexivCfg.xml
