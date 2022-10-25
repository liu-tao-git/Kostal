# slay app first
slay ArmDriver

# run ArmDriver
cp ArmDriver /tmp/
on -C 0 /tmp/ArmDriver -i8254x 1 1 -t 0 -dcmmode busshift -v 1 -eni eni_bub_bcb_abb_7x_wsb.xml -cfg A02LS-03-P2-062054/arm_driver_param.xml -sn A02LS-03-P2-062054 -lcns CX01-07-M2-062281.lic &
