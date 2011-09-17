#! /usr/bin/env python

import Hexapod
import time

if __name__ == '__main__':
    hex = Hexapod.Hexapod()
    
    js = Hexapod.RobotInput()
    js.start()
    
    try:
        hex.start()
        while True:
            hex.gait_engine.set_translation(js.y, js.x)
            hex.gait_engine.set_gait_rotation(-js.yaw*30)
            time.sleep(0.2)
        
    except KeyboardInterrupt:
        print('Received Keyboard Interrupt. Exiting.')
        
    hex.stop()
    js.stop()