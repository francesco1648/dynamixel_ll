import pygame
import serial
import time

# Inizializza Pygame e joystick
pygame.init()
pygame.joystick.init()

# Inizializza seriale (modifica con la porta corretta)
ser = serial.Serial('COM15', 115200)
time.sleep(2)

AXIS_THRESHOLD = 0.5

# Pulsanti
BUTTON_R1 = 9
BUTTON_R2 = 10
BUTTON_X = 0

# Pulsanti standard mappati
button_map = {
    2: 'z',  # Share
    3: 'x',  # Options
}

last_command = None

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Controller pronto. Premi R1, R2 o X + levetta sinistra...")

while True:
    pygame.event.pump()

    r1_active = joystick.get_button(BUTTON_R1)
    r2_active = joystick.get_button(BUTTON_R2)
    rx_active = joystick.get_button(BUTTON_X)

    x_axis = joystick.get_axis(0)
    y_axis = joystick.get_axis(1)

    command = None

    if r1_active:
        if y_axis < -AXIS_THRESHOLD:
            command = 'w'
        elif y_axis > AXIS_THRESHOLD:
            command = 's'
        elif x_axis > AXIS_THRESHOLD:
            command = 'd'
        elif x_axis < -AXIS_THRESHOLD:
            command = 'a'
    elif r2_active:
        if y_axis < -AXIS_THRESHOLD:
            command = 'y'
        elif y_axis > AXIS_THRESHOLD:
            command = 'h'
        elif x_axis > AXIS_THRESHOLD:
            command = 'j'
        elif x_axis < -AXIS_THRESHOLD:
            command = 'u'
    elif rx_active:
        if y_axis < -AXIS_THRESHOLD:
            command = 'i'
        elif y_axis > AXIS_THRESHOLD:
            command = 'k'
        elif x_axis > AXIS_THRESHOLD:
            command = 'o'
        elif x_axis < -AXIS_THRESHOLD:
            command = 'l'

    # SOLO se non ho ancora un comando, controllo gli altri pulsanti
    if command is None:
        for button in range(joystick.get_numbuttons()):
            if joystick.get_button(button):
                if button not in (BUTTON_R1, BUTTON_R2, BUTTON_X):
                    command = button_map.get(button)
                    break

    if command and command != last_command:
        ser.write(command.encode())
        print(f"Inviato: {command}")
        last_command = command

    if not command:
        last_command = None

    time.sleep(0.1)
