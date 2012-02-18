#! /usr/bin/env python

import Hexapod
import time

if __name__ == '__main__':
    hex = Hexapod.Hexapod()
    
    js = Hexapod.RobotInput(0.01)
    js.start()
    
    try:
        hex.start()
        while True:
            hex.gait_engine.set_translation(js.y, js.x)
            hex.gait_engine.set_gait_rotation(-js.yaw*30.0)
            hex.gait_engine.body_pos_offset[2] = 5.0 + (js.body_z+1.0) * 30.0
            
            hex.set_grip(70 - (js.grip+1) * 40)
            hex.set_gripper_angle( hex.gripper_angle + js.gripper_angle_delta)
            time.sleep(0.02)
        
    except KeyboardInterrupt:
        print('Received Keyboard Interrupt. Exiting.')
        
    hex.stop()
    js.stop()