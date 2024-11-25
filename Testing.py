import zmq
import msgpack
import numpy as np
import pygame.camera

pygame.init()
width, height = 1920, 1080
screen = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption('Gaze vizu')
clock = pygame.time.Clock()


overlay_w = 1420

smooth_pos = np.zeros(2)
smoothing = 60

April0 = 'April0.png'
April1 = 'April1.png'
April2 = 'April2.png'
April3 = 'April3.png'


pygame.camera.init()
cam = pygame.camera.Camera(0)
cam.start()

marker0 = pygame.image.load(April0)
marker1 = pygame.image.load(April1)
marker2 = pygame.image.load(April2)
marker3 = pygame.image.load(April3)

box1_size = 250
box2_size = 250
box3_size = 300
box4_size = 300

box0_act = (box3_size + 10, box1_size + 10, overlay_w-box3_size-box3_size-20, height-box1_size-box2_size-20)
box1_act = (box3_size+10, 10, overlay_w-box3_size-box4_size-20, box1_size-20)
box2_act = (box3_size+10, height-box2_size+10, overlay_w-box3_size-box4_size-20, box2_size-20)
box3_act = (10, box1_size+10, box3_size-20, height-box1_size-box2_size-20)
box4_act = (overlay_w-box4_size+10, box1_size+10, box4_size-20, height-box1_size-box2_size-20)

box1 = (box3_size, 0, overlay_w-box3_size-box4_size, box1_size)
box2 = (box3_size, height-box2_size, overlay_w-box3_size-box4_size, box2_size)
box3 = (0, box1_size, box3_size, height-box1_size-box2_size)
box4 = (overlay_w-box4_size, box1_size, box4_size, height-box1_size-box2_size)

Color1 = (171,194,196,128)
Color2 = (171,194,196,128)
Color3 = (171,194,196,128)
Color4 = (171,194,196,128)

actuation = (overlay_w+15, height/4, width-overlay_w-30, height/2)
actuation_outline = (overlay_w+15, height/4, width-overlay_w-30, height/2)

box = None
locked_box = None

ctx = zmq.Context()

req = zmq.Socket(ctx, zmq.REQ)
ip = 'localhost'
port = 50020

#req.connect(f'tcp://{ip}:{port}')
#try:
   #req.send_string('SUB_PORT')
   #sub_port = req.recv_string()  # this is the line where everything crash
#except zmq.ZMQError as e:
    #print(f'Error port: {e}')
    #running = False

#surf_sub = ctx.socket(zmq.SUB)
#surf_sub.connect(f'tcp://{ip}:{sub_port}')
#surf_sub.subscribe('surfaces.Surface 1')

#surf_sub.setsockopt(zmq.RCVTIMEO, 1000)

running = True
gaze_pos_surf = None
gaze_pos_his = []


Menu = 0


#def recv_surf_data():
    #global gaze_pos_surf, smooth_pos
    #try:
        #topic, payload = surf_sub.recv_multipart()
        #message = msgpack.loads(payload)
        #print(f"{topic}:{message}")

        #if b'gaze_on_surfaces' in message:
            #gaze_on_surface = message[b'gaze_on_surfaces']

            #for gaze in gaze_on_surface:
                #if gaze[b'on_surf']:
                    #gaze_pos_surf = gaze[b'norm_pos']

                    #gaze_pos_his.append(gaze_pos_surf)

                    #if len(gaze_pos_his) > smoothing:
                        #gaze_pos_his.pop(0)
                    #if len(gaze_pos_his)%5 == 0:
                        #smooth_pos[:] = np.mean(gaze_pos_his, axis=0)
    #except zmq.Again:
        #pass


#req.send_string('C')
#print(req.recv_string())

def DrawGazeSurface():
    
    #Draws the gaze surface on the screen with specified boxes and lines.
    #The function performs the following drawing operations:
   
    #Draws Box1 - Box4 with a rectangle of color (171,194,196).
    Box1_transparent = pygame.Surface((box1[2], box1[3]), pygame.SRCALPHA)
    Box2_transparent = pygame.Surface((box2[2], box2[3]), pygame.SRCALPHA)
    Box3_transparent = pygame.Surface((box3[2], box3[3]), pygame.SRCALPHA)
    Box4_transparent = pygame.Surface((box4[2], box4[3]), pygame.SRCALPHA)

    pygame.draw.rect(Box1_transparent, (Color1), (0, 0, box1[2], box1[3]), 0)
    pygame.draw.rect(Box2_transparent, (Color2), (0, 0, box2[2], box2[3]), 0)
    pygame.draw.rect(Box3_transparent, (Color3), (0, 0, box3[2], box3[3]), 0)
    pygame.draw.rect(Box4_transparent, (Color4), (0, 0, box4[2], box4[3]), 0)

    screen.blit(Box1_transparent, (box1[0], box1[1]))
    screen.blit(Box2_transparent, (box2[0], box2[1]))
    screen.blit(Box3_transparent, (box3[0], box3[1]))
    screen.blit(Box4_transparent, (box4[0], box4[1]))

    #Draws a horizontal line at the bottom of Box1 with color (0,15,8).
    pygame.draw.line(screen, (0,15,8), (0, box1_size),(overlay_w, box1_size), 3)

    #Draws a horizontal line at the top of Box2 with color (0,15,8).
    pygame.draw.line(screen, (0,15,8), (0, height-box2_size),(overlay_w, height-box2_size), 3)

    #Draws a vertical line at the right of Box3 with color (0,15,8).
    pygame.draw.line(screen, (0,15,8), (box3_size, 0),(box3_size, height), 3)

    #Draws a vertical line at the left of Box4 with color (0,15,8).
    pygame.draw.line(screen, (0,15,8), (overlay_w-box4_size, 0),(overlay_w-box4_size, height), 3)

    #Draws a vertical line at the right edge of the overlay with color (0,15,8).


def DrawTongueViz():
    pygame.draw.rect(screen, (0, 15,8), (actuation_outline), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w+15, (height/4)*2.3), (width-16, (height/4)*2.3), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w + (width-overlay_w)/2, (height/4)*2.3), (overlay_w + (width-overlay_w)/2, ((height/4)*3)-1), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w+15+115.3, height/4), (overlay_w+15+115.3, (height/4)*2.3), 5)
    pygame.draw.line(screen, (0,0,0), (width-15-115.3, height/4), (width-15-115.3, (height/4)*2.3), 5)


def CallCalibration():
    if keys[pygame.K_c]:
            req.close()  # Close the existing socket
            req = ctx.socket(zmq.REQ)  # Reinitialize the socket
            req.connect(f'tcp://{ip}:{port}')
            req.send_string('C')


def DrawMenu(Menu):

    pygame.draw.rect(screen, (0,15,8), (overlay_w + 1 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 25 ,height-150, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 2 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 25,height-150, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 3 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 25,height-150, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 4 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 25,height-150, 50, 75), 3)

    ArrowUp = pygame.image.load('arrow_up.png')
    ArrowDown = pygame.image.load('arrow_down.png')
    ArrowLeft = pygame.image.load('arrow_left.png')
    ArrowRight = pygame.image.load('arrow_right.png')
    
    Up = pygame.image.load('rotate_U.png')
    Down = pygame.image.load('rotate_D.png')
    Right = pygame.image.load('rotate_R.png')
    Left = pygame.image.load('rotate_L.png')

    missing = pygame.image.load('missing.jpg')

    Forward = pygame.image.load('Forward150.png')
    Back = pygame.image.load('Back.png')
    U = pygame.image.load('U.png')
    D = pygame.image.load('D.png')

    if Menu == 0:
        
        screen.blit(ArrowUp, (overlay_w/2 -190/2, box1_size/2 -190/2))
        screen.blit(ArrowDown, (overlay_w/2 -190/2, height-(box2_size/2) -190/2))
        screen.blit(ArrowLeft, (box3_size/2 -190/2, height/2 -190/2))
        screen.blit(ArrowRight, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 1 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 23 ,height-148, 46, 71), 0)
    elif Menu == 1:
        
        screen.blit(Up, (overlay_w/2 -190/2, box1_size/2 -190/2 + 20))
        screen.blit(Down, (overlay_w/2 -190/2, height-(box2_size/2) -190/2 + 20))
        screen.blit(Right, (overlay_w-box4_size/2 -190/2 + 50, height/2 -190/2))
        screen.blit(Left, (box3_size/2 -190/2, height/2 -190/2))
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 2 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 23,height-148, 46, 71), 0)

    elif Menu == 2:
        
        screen.blit(ArrowUp, (overlay_w/2 -190/2 , box1_size/2 -190/2 -20))
        screen.blit(Forward, (overlay_w/2 -150/2 -6 , box1_size - 40 ))

        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Back, (overlay_w/2 -93/2 + 6 , height -40))

        screen.blit(ArrowUp, (overlay_w-box4_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(U, (overlay_w-box4_size/2 -28/2 - 5 , height/2 + 90))
        
        screen.blit(ArrowDown, (box3_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(D, (box3_size/2 -28/2 +5 , height/2 + 90))

        pygame.draw.rect(screen,(0,255,0), (overlay_w + 3 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 23,height-148, 46, 71), 0)
    elif Menu == 3:
        
        screen.blit(missing, (overlay_w/2 -190/2, box1_size/2 -190/2))
        screen.blit(missing, (overlay_w/2 -190/2, height-(box2_size/2) -190/2))
        screen.blit(missing, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        screen.blit(missing, (box3_size/2 -190/2, height/2 -190/2))
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 4 * (width-overlay_w)/4 - (1 * (width-overlay_w)/4)/2 - 23,height-148, 46, 71), 0)
    else:
        pass
    

def VisInput():
    if keys[pygame.K_KP4]: 
        pygame.draw.circle(screen, (0,255,0), (overlay_w+68, height/2.5), 20)
    
    if keys[pygame.K_KP5]:
        pygame.draw.circle(screen, (0,255,0), (width-((width-overlay_w)/2), height/2.5), 20)

    if keys[pygame.K_KP6]:
        pygame.draw.circle(screen, (0,255,0), (width-68, height/2.5), 20)

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        

        #initialize keyboard input
        keys = pygame.key.get_pressed()

        #set frame rate
        clock.tick(30)
        
        #draw background
        screen.fill((255,255,255))

        img = cam.get_image()
        img = pygame.transform.scale(img, (overlay_w, height))
        screen.blit(img, (0,0))

        #draw boxes
        DrawGazeSurface()
        
        #Draw markers for surface tracking
        screen.blit(marker0, (0,0))
        screen.blit(marker1, (0, height-207))
        screen.blit(marker2, (overlay_w-207, height-207))
        screen.blit(marker3, (overlay_w-204, 0))


        #recv_surf_data()
        
        #Calculate gaze position if gaze_pos_surf is not None
        if gaze_pos_surf is not None:
            x_pixel = int(gaze_pos_surf[0] * overlay_w)
            y_pixel = height - int(gaze_pos_surf[1] * height)
            # print('x,y', x_pixel, y_pixel)
            x_smooth = int(smooth_pos[0] * overlay_w)
            y_smooth = height - int(smooth_pos[1] * height)

            
            if not keys[pygame.K_UP]:
                transparent_circle_smooth = pygame.Surface((40, 40), pygame.SRCALPHA)
                pygame.draw.circle(transparent_circle_smooth, (0, 255, 0, 128), (20, 20), 20)  # Semi-transparent green
                screen.blit(transparent_circle_smooth, (x_smooth - 20, y_smooth - 20))

                transparent_circle_pixel = pygame.Surface((40, 40), pygame.SRCALPHA)
                pygame.draw.circle(transparent_circle_pixel, (255, 0, 0, 128), (20, 20), 20)  # Semi-transparent red
                screen.blit(transparent_circle_pixel, (x_pixel - 20, y_pixel - 20))

                if box3_size < x_pixel < overlay_w-box4_size and y_pixel < box1_size:
                    box = 1
                    Color1 = (0,255,0,128)
                elif box3_size < x_pixel < overlay_w-box4_size and y_pixel > height-box2_size:
                    box = 2
                    Color2 = (0,255,0,128)
                elif box1_size < y_pixel < height-box2_size and x_pixel < box3_size:
                    box = 3
                    Color3 = (0,255,0,128)
                elif box1_size < y_pixel < height-box2_size and x_pixel > overlay_w-box4_size:
                    box = 4
                    Color4 = (0,255,0,128)
                else:
                    box = 0
                    Color1 = (171,194,196,128)
                    Color2 = (171,194,196,128)
                    Color3 = (171,194,196,128)  
                    Color4 = (171,194,196,128)


        pygame.draw.line(screen, (0,15,8), (overlay_w,0+1),(overlay_w, height), 5)

        DrawTongueViz()

        

        CallCalibration()
            
        
        
        DrawMenu(Menu)
        
        if keys[pygame.K_1]:
            Menu = 0
        
        if keys[pygame.K_2]:
            Menu = 1
        
        if keys[pygame.K_3]:
            Menu = 2
        
        if keys[pygame.K_4]:
            Menu = 3
        
        VisInput()

        if keys[pygame.K_ESCAPE]:
            running = False

        if keys[pygame.K_UP]:
            pygame.draw.rect(screen, (0,255,0), (actuation), 0)
            locked_box = box
        else:
            locked_box = None

        if locked_box == 0:
            pygame.draw.rect(screen,(0,255,0), (box0_act), 5)
        if locked_box == 1:
            pygame.draw.rect(screen, (0,255,0), (box1_act), 5)
        if locked_box == 2:
            pygame.draw.rect(screen,(0,255,0), (box2_act), 5)
        if locked_box == 3:
            pygame.draw.rect(screen,(0,255,0), (box3_act), 5)
        if locked_box == 4:
            pygame.draw.rect(screen,(0,255,0), (box4_act), 5)
        
        pygame.display.flip()



except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
