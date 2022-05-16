#!/usr/bin/env python3

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

def main():
  rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
  rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
  speed = [0, 0, 0.100, 0, 0, 0]
  rtde_c.moveUntilContact(speed)

  rtde_c.stopScript()


if __name__ == '__main__':
  main()
