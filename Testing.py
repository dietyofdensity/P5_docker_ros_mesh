import zmq
import msgpack
import numpy as np
import pygame

pygame.init()
width,height = 1920,1080
screen = pygame.display.set_mode((width,height), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption('Gaze vizu')
clock = pygame.time.Clock()

smoothing = 60

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

surf_sub.setsockopt(zmq.RCVTIMEO, 1000)

running = True
gaze_pos_surf = None
gaze_pos_his = []

def recv_surf_data():
    global gaze_pos_surf
    try:
        topic, payload = surf_sub.recv_multipart()
        message = msgpack.loads(payload)
        #print(f"{topic}:{message}")

        if b'gaze_on_surfaces' in message:
            gaze_on_surface = message[b'gaze_on_surfaces']

            for gaze in gaze_on_surface:
                if gaze[b'on_surf']:
                    gaze_pos_surf = gaze[b'norm_pos']

                    gaze_pos_his.append(gaze_pos_surf)

                    if len(gaze_pos_his) > smoothing:
                        gaze_pos_his.pop(0)          
    except zmq.Again:
        pass

def get_smoothed_pos():
    if len(gaze_pos_his) == 0:
        return None
    
    smooth_pos = np.mean(gaze_pos_his, axis=0)
    return smooth_pos
    

try:

    screen.fill((255,255,255))
        
    screen.blit(marker0, (0,0))
    screen.blit(marker1, (0, height-207))
    screen.blit(marker2, (width-207, height-207))
    screen.blit(marker3, (width-204, 0))

    pygame.display.flip()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        recv_surf_data()

        screen.fill((255,255,255))
        
        screen.blit(marker0, (0,0))
        screen.blit(marker1, (0, height-207))
        screen.blit(marker2, (width-207, height-207))
        screen.blit(marker3, (width-204, 0))

        if gaze_pos_surf is not None:
                gaze_smooth = get_smoothed_pos()
                x_pixel = int(gaze_pos_surf[0] * width)
                y_pixel = height - int(gaze_pos_surf[1] * height)
                #print('x,y', x_pixel, y_pixel)
                x_smooth = int(gaze_smooth[0] * width)
                y_smooth = height - int(gaze_smooth[1] * height)
                

                pygame.draw.circle(screen, (0,255,0), (x_smooth,y_smooth), 20)
                pygame.draw.circle(screen, (255,0,0), (x_pixel, y_pixel), 20)
                
        pygame.draw.circle(screen, (0,0,255), (104, height/2), 10)
        pygame.draw.circle(screen, (0,0,255), (width-104, height/2), 10)
        pygame.draw.circle(screen, (0,0,255), (width/2, 104), 10)
        pygame.draw.circle(screen, (0,0,255), (width/2, height-104), 10)
        pygame.draw.circle(screen, (0,0,255), (width/2, height/2), 10)
        pygame.draw.line(screen, (0,255,255), (0,200),(width, 200), 2)
        pygame.draw.line(screen, (0,255,255), (0,height-200),(width, height-200), 2)
        pygame.draw.line(screen, (0,255,255), (300,0),(300, height), 2)
        pygame.draw.line(screen, (0,255,255), (width-300,0),(width-300, height), 2)
        pygame.display.flip()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            running = False


except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
    
