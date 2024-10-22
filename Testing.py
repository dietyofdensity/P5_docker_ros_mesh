import zmq
import msgpack
import numpy as np
import pygame

pygame.init()
width,height = 1920,1080
screen = pygame.display.set_mode((width,height), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption('Gaze vizu')
clock = pygame.time.Clock()



April0 = 'April0.png'
April1 = 'April1.png'
April2 = 'April2.png'
April3 = 'April3.png'

marker0 = pygame.image.load(April0)
marker1 = pygame.image.load(April1)
marker2 = pygame.image.load(April2)
marker3 = pygame.image.load(April3)

ctx = zmq.Context()

req = zmq.Socket(ctx, zmq.REQ)
ip = 'localhost'
port = 50020

req.connect(f'tcp://{ip}:{port}')

req.send_string('SUB_PORT')
sub_port = req.recv_string()

gaze_sub = ctx.socket(zmq.SUB)
gaze_sub.connect(f'tcp://{ip}:{sub_port}')
gaze_sub.subscribe('gaze.2d.0')

surf_sub = ctx.socket(zmq.SUB)
surf_sub.connect(f'tcp://{ip}:{sub_port}')
surf_sub.subscribe('surfaces.Surface 1')

norm_pos = None
base_norm_pos = None

running = True
gaze_pos_surf = None

def recv_surf_data():
    global gaze_pos_surf, surf_to_img
    topic, payload = surf_sub.recv_multipart()
    message = msgpack.loads(payload)
    #print(f"{topic}:{message}")

    if b'surf_to_img_trans' in message:
        surf_to_img = message[b'surf_to_img_trans']
        surf_to_img = np.array(surf_to_img)

    if b'gaze_on_surfaces' in message:
        gaze_on_surface = message[b'gaze_on_surfaces']

        for gaze in gaze_on_surface:
            if gaze[b'on_surf']:
                gaze_pos_surf = gaze[b'norm_pos']
                

    

try:

    screen.fill((255,255,255))
        
    screen.blit(marker0, (0,0))
    screen.blit(marker1, (0, 873))
    screen.blit(marker2, (1713, 873))
    screen.blit(marker3, (1713, 0))

    pygame.display.flip()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        recv_surf_data()

        screen.fill((255,255,255))
        
        screen.blit(marker0, (0,0))
        screen.blit(marker1, (0, 873))
        screen.blit(marker2, (1713, 873))
        screen.blit(marker3, (1713, 0))

        if gaze_pos_surf is not None:
                x_pixel = int(gaze_pos_surf[0] * width)
                y_pixel = height - int(gaze_pos_surf[1] * height)
                #print('x,y', x_pixel, y_pixel)
                

                gaze_pose_homogenous = np.array([gaze_pos_surf[0], gaze_pos_surf[1],1])
                trans_pos = np.dot(surf_to_img, gaze_pose_homogenous)
                trans_pos = trans_pos[:2]/trans_pos[2]

                print(trans_pos)
                pygame.draw.circle(screen, (0,255,0), trans_pos, 20)
                pygame.draw.circle(screen, (255,0,0), (x_pixel, y_pixel), 20)
                
        pygame.draw.circle(screen, (0,0,255), (104, 540), 10)
        pygame.draw.circle(screen, (0,0,255), (1816, 540), 10)
        pygame.draw.circle(screen, (0,0,255), (960, 104), 10)
        pygame.draw.circle(screen, (0,0,255), (960, 976), 10)
        pygame.draw.circle(screen, (0,0,255), (960, 540), 10)
        pygame.display.flip()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            running = False


except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    
