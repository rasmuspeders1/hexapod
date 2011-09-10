'''
Created on Sep 10, 2011

@author: rasmus
'''

import Hexapod

if __name__ == '__main__':
    hex = Hexapod.hexapod()
    
    try:
        hex.start()
        raw_input()
        hex.stop()
        
    except KeyboardInterrupt:
        print('Received Keyboard Interrupt. Exiting.')
        hex.stop()