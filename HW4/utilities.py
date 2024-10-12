# Utilities including plotting
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

import math
import pygame

WHITE = (255,255,255)
COLOR = (255, 200, 0)

# Drone dimensions
body_width = 150
body_height = 15
shaft_width = 5
shaft_height = 35
propeller_width = 70
propeller_height = 10
propeller_offset = 55  # Distance from the body center to the propellers

# draw everything
def draw(screen,font, quad1, desired_angle, thrust_input, running, simTime, clock, m1_log, m2_log):
    info = pygame.display.Info()
    window_width = info.current_w
    window_height = info.current_h

    screen.fill((0,0,40))

    # Draw the drone with the current rotation angle on the screen
    draw_drone(screen, window_width // 2 + quad1.x*50, window_height // 2 - quad1.y*50, -quad1.phi)

    # write the texts
    if running:
        running_text = font.render('SIM RUNNING', True, (50, 255, 50))
    else:
        running_text = font.render('SIM STOPPED', True, (255, 50, 50))
    screen.blit(running_text, (10, 10))

    run_time_text = font.render(f'Run time: {round(simTime)}', True, (255, 255, 255))
    frame_rate_text = font.render(f'Frame rate: {round(clock.get_fps())}', True, (255, 255, 255))
    screen.blit(run_time_text, (10, 30))
    screen.blit(frame_rate_text, (10, 50))
    info_text1 = font.render('Space: run/stop', True, (255, 255, 255))
    info_text2 = font.render('Mouse position: desired angle and thrust', True, (255, 255, 255))
    screen.blit(info_text1, (10, 100))
    screen.blit(info_text2, (10, 120))

    # draw setpoints
    pygame.draw.aaline(screen, (200, 20, 20),
                   (window_width / 2 + quad1.x * 50 - body_width * math.cos(desired_angle), window_height / 2 - quad1.y * 50 - body_width * math.sin(desired_angle)),
                     (window_width / 2 + quad1.x * 50 + body_width * math.cos(desired_angle), window_height / 2 - quad1.y * 50 + body_width * math.sin(desired_angle)))
    pygame.draw.aaline(screen, (200, 20, 20),
                     (0, window_height / 2 - 50 * thrust_input),
                     (window_width, window_height / 2 - 50 * thrust_input))

    # draw the center
    pygame.draw.circle(screen, (255,0,0), (window_width//2, window_height//2), 3)    

    # draw the plot
    pygame.draw.aaline(screen, (255,255,255), (10, 270), (280, 270))  # X-axis
    pygame.draw.aaline(screen, (255,255,255), (40, 200), (40, 340))   # Y-axis
    y_label = font.render("motor commands", True, (255,255,255))
    screen.blit(y_label, (65, 170))             # Y-axis label

    scale = 0.4
    for i in range(1, len(m1_log)):
        pygame.draw.aaline(screen, (155,50,50),
                        ((i - 1) + 40, 270 - m1_log[i - 1] * scale),
                         (i + 40, 270 - m1_log[i] * scale))
        pygame.draw.aaline(screen, (255,255,255),
                        ((i - 1) + 40, 270 - m2_log[i - 1] * scale),
                         (i + 40, 270 - m2_log[i] * scale))



def draw_drone(surface, xpos, ypos, angle):

    # Create a surface
    drone_surf = pygame.Surface((propeller_offset*2+propeller_width, shaft_height + body_height))
    drone_surf.set_colorkey((0, 0, 0))  # Set black as transparent color

    # Calculate the center of the drone surface
    center_x = drone_surf.get_width() // 2
    center_y = drone_surf.get_height() // 2

    # Draw the body of the drone
    body_rect = pygame.Rect(center_x - body_width // 2, center_y - body_height // 2, body_width, body_height)
    pygame.draw.rect(drone_surf, WHITE, body_rect)

    # Draw the left shaft
    left_shaft_rect = pygame.Rect(center_x - propeller_offset - shaft_width // 2, center_y - shaft_height + body_height // 2, shaft_width, shaft_height)
    pygame.draw.rect(drone_surf, WHITE, left_shaft_rect)

    # Draw the right shaft
    right_shaft_rect = pygame.Rect(center_x + propeller_offset - shaft_width // 2, center_y - shaft_height + body_height // 2, shaft_width, shaft_height)
    pygame.draw.rect(drone_surf, WHITE, right_shaft_rect)

    # Draw the propellers as ovals
    left_propeller_rect = pygame.Rect(0, 0, propeller_width, propeller_height)
    right_propeller_rect = pygame.Rect(center_x + propeller_offset - propeller_width // 2, 0, propeller_width, propeller_height)

    pygame.draw.ellipse(drone_surf, COLOR, left_propeller_rect)
    pygame.draw.ellipse(drone_surf, COLOR, right_propeller_rect)

    # Rotate the entire drone surface
    rotated_drone_surf = pygame.transform.rotate(drone_surf, math.degrees(angle))
    rotated_rect = rotated_drone_surf.get_rect(center=(xpos, ypos))

    # Blit the rotated drone to the screen
    surface.blit(rotated_drone_surf, rotated_rect.topleft)

# Mouse mapping. Around 2 pixels from the center: 0 input. After that quadratic function 
# (to ensure high precision around the center) followed by a linear function.
# No discontinuities during the transitions
def remapMouse(reading, window_width):
    x1 = 2
    x2 = 100
    out = 0

    if x1 <= reading < x2:
        out = 1 / 2 / (x2 - x1) * (reading**2 - 2 * x1 * reading + x1 * x1)
    elif reading >= x2:
        out = reading - x2 / 2 - x1 / 2

    x1 = -x1
    x2 = -x2
    if x1 >= reading > x2:
        out = 1 / 2 / (x2 - x1) * (reading**2 - 2 * x1 * reading + x1 * x1)
    elif reading <= x2:
        out = reading - x2 / 2 - x1 / 2

    out = min(window_width / 4, max(out, -window_width / 4))
    return out
