import winsound
import pygame

running = True

# winsound.PlaySound("C:/Users/FRC1073/Desktop/Mario-coin-sound.wav", winsound.SND_FILENAME)

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
	print("No joysticks found.")
	pygame.quit()
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick {joystick.get_name()} initialized")
while running:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
		elif event.type == pygame.JOYBUTTONDOWN:
			print("Button pressed:", event.button)
		elif event.type == pygame.JOYBUTTONUP:
			print("Button released:", event.button)
		elif event.type == pygame.JOYAXISMOTION:
			print("Axis moved:", event.axis, "Value:", event.value)
		elif event.type == pygame.JOYHATMOTION:
			print("Hat switch moved:", event.hat, "Value:", event.value)
pygame.quit()