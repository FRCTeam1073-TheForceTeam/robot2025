# import winsound
import pygame

running = True

# winsound.PlaySound("C:/Users/FRC1073/Desktop/Mario-coin-sound.wav", winsound.SND_FILENAME)
# pygame docs: https://www.pygame.org/docs/ref/joystick.html
# work for bluetooth connected controller but not hard connect

pygame.init()
pygame.joystick.init()
pygame.mixer.init()
sound = pygame.mixer.Sound("/Users/williamvanuitert/Desktop/Mario-coin-sound.mp3")


joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
	print("No joysticks found.")
	pygame.QUIT
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick {joystick.get_name()} initialized")

while running:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
		if event.type == pygame.JOYBUTTONDOWN:
			print("Button pressed:", event.button)

			if(event.button == 7):
				sound.play()

		if event.type == pygame.JOYBUTTONUP:
			print("Button released:", event.button)

		if event.type == pygame.JOYAXISMOTION and abs(event.value) > 0.5:
			print("Axis moved:", event.axis, "Value:", event.value)

		if event.type == pygame.JOYHATMOTION:
			print("Hat switch moved:", event.hat, "Value:", event.value)

pygame.QUIT
