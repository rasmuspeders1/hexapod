#! /usr/bin/env python

import Hexapod

import time

if __name__ == '__main__':
    hex = Hexapod.Hexapod()
    
    try:
        hex.gait_engine.set_gait_direction(1, 0)
        hex.gait_engine.set_gait_rotation(0)
        hex.start()
        while hex.gait_engine.cycle < 7:
            time.sleep(0.2)
        
        hex.gait_engine.set_gait_direction(0, 0)
        hex.gait_engine.set_gait_rotation(-15)
        while hex.gait_engine.cycle < 11:
            time.sleep(0.2)        
        
        hex.gait_engine.set_gait_direction(1, 0)
        hex.gait_engine.set_gait_rotation(0)
        while hex.gait_engine.cycle < 16:
            time.sleep(0.2)
        
        hex.gait_engine.set_gait_direction(1, 0)
        hex.gait_engine.set_gait_rotation(15)
        while hex.gait_engine.cycle < 26:
            time.sleep(0.2)
        
        hex.stop()
        
    except KeyboardInterrupt:
        print('Received Keyboard Interrupt. Exiting.')
        hex.stop()