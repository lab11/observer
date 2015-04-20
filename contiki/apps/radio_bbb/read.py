import sys
import pygame
from pygame.locals import *

# Initialize Demo
pygame.init()
width, height = 1024, 768
screen=pygame.display.set_mode((width, height))

# Load Images
background = pygame.image.load("resources_bbb/images_2/template_6.png")

font = pygame.font.Font(None, 50)

background_rect = background.get_rect()
screen.blit(background, background_rect)

while 1:
    data = sys.stdin.readline()
    # do stuff with data
    parsed_data = data.split(' ')
    # parsed_data[0] is a space/null character
    # it's there b/c of the way read.c prints out each byte with space beforehand
    print parsed_data[0:4]
    print parsed_data[4]
    print parsed_data[5]
    print parsed_data[6]
    print parsed_data[7]
    parsed_data[7] = parsed_data[7].strip()
    screen.blit(background, background_rect)
    accelx_text = font.render(str(parsed_data[0]), True, (0,0,0))
    accely_text = font.render(str(parsed_data[1]), True, (0,0,0))
    accelz_text = font.render(str(parsed_data[2]), True, (0,0,0))
    humid_text = font.render(str(parsed_data[3]), True, (0,0,0))
    light_text = font.render(str(parsed_data[4]), True, (0,0,0))
    mic_text = font.render(str(parsed_data[5]), True, (0,0,0))
    press_text = font.render(str(parsed_data[6]), True, (0,0,0))
    temp_text = font.render(str(parsed_data[7]), True, (0,0,0))
    accelx_rect = accelx_text.get_rect()
    accely_rect = accely_text.get_rect()
    accelz_rect = accelz_text.get_rect()
    humid_rect = humid_text.get_rect()
    light_rect = light_text.get_rect()
    mic_rect = mic_text.get_rect()
    press_rect = press_text.get_rect()
    temp_rect = temp_text.get_rect()    
    accelx_rect.topleft=[358, 700]
    accely_rect.topleft=[472, 700]
    accelz_rect.topleft=[586, 700]
    humid_rect.topleft=[477, 316]
    light_rect.topleft=[131, 700]
    mic_rect.topleft=[813, 700]
    press_rect.topleft=[813, 316]
    temp_rect.topleft=[136, 316]
    screen.blit(accelx_text, accelx_rect)
    screen.blit(accely_text, accely_rect)
    screen.blit(accelz_text, accelz_rect)
    screen.blit(humid_text, humid_rect)
    screen.blit(light_text, light_rect)
    screen.blit(mic_text, mic_rect)
    screen.blit(press_text, press_rect)
    screen.blit(temp_text, temp_rect)

    pygame.display.flip()
    
        
