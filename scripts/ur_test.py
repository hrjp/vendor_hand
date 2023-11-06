import rtde_receive
import rtde_control
rtde_c = rtde_control.RTDEControlInterface("192.168.0.119")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.119")
actual = rtde_r.getActualTCPPose()
actual[2]-=0.1
rtde_c.moveL(actual, 0.1, 0.3)
print(actual)
actual[2]+=0.1
rtde_c.moveL(actual, 0.1, 0.3)
print(actual)
rtde_c.stopScript()