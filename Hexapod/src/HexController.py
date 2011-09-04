import pygame
import threading
from pygame.locals import *

class HexController(threading.Thread):
    
    def __init__(self):
        # Initialize PyGame
        pygame.init()
        pygame.display.set_caption('Hexapod Controller')
        self.screen = pygame.display.set_mode((400,400))
        self.screen.fill((0,0,0))
        self.pos = (0,0)
        self.rot = (0,0)
        threading.Thread.__init__(self)  
        self.daemon = True   
        

    def run(self):
        # Update display initially
        pygame.display.update()
        # Run the event loop
        self.loop()
        # Close the Pygame window
        pygame.quit()
    
    def loop(self):
        exit = False
        while not exit:
            # Handle input events
            exit = self.handleEvents()
            # Wait a little?
            pygame.time.delay(10)

    def handleEvents(self):
            exit = False
            for event in pygame.event.get():
                if event.type == QUIT:
                    exit = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        exit = True
                elif event.type == MOUSEMOTION:
                    b1,b2,b3 = pygame.mouse.get_pressed()
                    if b1 == True:
                        self.pos = pygame.mouse.get_pos()
                    if b3 == True:
                        self.rot = pygame.mouse.get_pos()
                        
            return exit
        

  