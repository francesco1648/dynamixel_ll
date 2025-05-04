import pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Premi un pulsante sul controller...")

while True:
    pygame.event.pump()
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            print(f"Pulsante premuto: {i}")
