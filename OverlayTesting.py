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

Simulate = False
Y_sim = 0
X_sim = 0

box = None
locked_box = None

ctx = zmq.Context()

running = True

Menu = 0


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


def DrawMenu(Menu):

    pygame.draw.rect(screen, (0,15,8), (overlay_w + 1 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25 ,height-120, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 2 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 3 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 4 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3)
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 5 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3)

    ArrowUp = pygame.image.load('arrow_up.png')
    ArrowDown = pygame.image.load('arrow_down.png')
    ArrowLeft = pygame.image.load('arrow_left.png')
    ArrowRight = pygame.image.load('arrow_right.png')
    
    R_Up = pygame.image.load('rotate_U.png')
    R_Down = pygame.image.load('rotate_D.png')
    R_Right = pygame.image.load('rotate_R.png')
    R_Left = pygame.image.load('rotate_L.png')

    missing = pygame.image.load('missing.jpg')

    Forward = pygame.image.load('Forward150.png')
    Back = pygame.image.load('Back.png')
    Up = pygame.image.load('UP.png')
    Down = pygame.image.load('Down.png')
    Left = pygame.image.load('LEFT.png')
    Right = pygame.image.load('RIGHT.png')

    PanU = pygame.image.load('PanU.png')
    PanD = pygame.image.load('PanD.png')
    PanL = pygame.image.load('PanL.png')
    PanR = pygame.image.load('PanR.png')
    
    Roll_L = pygame.image.load('Roll_L.png')
    Roll_R = pygame.image.load('Roll_R.png')

    PivotL = pygame.image.load('PivotL.png')
    PivotR = pygame.image.load('PivotR.png')
    PLeft = pygame.image.load('PLeft.png')
    PRight = pygame.image.load('PRight.png')


    Trans = pygame.image.load('Translational.png')
    Rot = pygame.image.load('Rotational.png')
    BaseHand = pygame.image.load('BaseHand.png')
    ForwardRoll = pygame.image.load('ForwardRoll.png')
    ForwardVertical = pygame.image.load('ForwardVert.png')

    if Menu == 0:
        
        screen.blit(ArrowUp, (overlay_w/2 -190/2, box1_size/2 -190/2 + 20))
        screen.blit(Up, (overlay_w/2 -62/2 - 5 , 10))

        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Down, (overlay_w/2 -115/2 + 6 , height -40))

        screen.blit(ArrowLeft, (box3_size/2 -190/2, height/2 -190/2))
        screen.blit(Left, (box3_size/2 -115/2 +5 , height/2 + 110))

        screen.blit(ArrowRight, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        screen.blit(Right, (overlay_w-box4_size/2 -115/2 - 5 , height/2 + 110))
    
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 1 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23 ,height-118, 46, 71), 0)
        screen.blit(Trans,(overlay_w + (width-overlay_w)/2 - 282/2, height-170))

    elif Menu == 1:
        
        screen.blit(R_Up, (overlay_w/2 -190/2, box1_size/2 -190/2 + 30))
        screen.blit(PanU, (overlay_w/2 -145/2, 10))

        screen.blit(R_Down, (overlay_w/2 -190/2, height-(box2_size/2) -190/2))
        screen.blit(PanD, (overlay_w/2 -211/2, height - 40))

        screen.blit(R_Right, (overlay_w-box4_size/2 -190/2 + 50, height/2 -190/2 - 10))
        screen.blit(PanR, (overlay_w-box4_size/2 -215/2, height/2 + 110))

        screen.blit(R_Left, (box3_size/2 -190/2, height/2 -190/2 - 10))
        screen.blit(PanL, (box3_size/2 -190/2, height/2 + 110))

        pygame.draw.rect(screen,(0,255,0), (overlay_w + 2 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(Rot,(overlay_w + (width-overlay_w)/2 - 306/2, height-170))

    elif Menu == 2:
        
        screen.blit(ArrowUp, (overlay_w/2 -190/2 , box1_size/2 -190/2 + 20))
        screen.blit(Forward, (overlay_w/2 -150/2 -6 , 10 ))

        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Back, (overlay_w/2 -93/2 + 6 , height -40))

        screen.blit(R_Left, (overlay_w-box4_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(Roll_L, (overlay_w-box4_size/2 -215/2 , height/2 + 110))
        
        screen.blit(R_Right, (box3_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(Roll_R, (box3_size/2 -239/2 , height/2 + 110))

        pygame.draw.rect(screen,(0,255,0), (overlay_w + 3 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(ForwardRoll,(overlay_w + (width-overlay_w)/2 - 266/2, height-170))
    elif Menu == 3:
        
        screen.blit(ArrowUp, (overlay_w/2 -190/2 , box1_size/2 -190/2 + 20))
        screen.blit(Forward, (overlay_w/2 -150/2 -6 , 10 ))
        
        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Back, (overlay_w/2 -93/2 + 6 , height -40))

        screen.blit(ArrowDown, (box3_size/2 -(190/2) - 15, height/2 -190/2))
        screen.blit(Down, (box3_size/2 -115/2 - 5 , height/2 + 110))

        screen.blit(ArrowUp, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        screen.blit(Up, (overlay_w-box4_size/2 -62/2 - 5 , height/2 + 110))
    

        pygame.draw.rect(screen,(0,255,0), (overlay_w + 4 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(ForwardVertical,(overlay_w + (width-overlay_w)/2 - 355/2, height-170))
    
    elif Menu == 4:
        screen.blit(ArrowUp, (overlay_w/2 -190/2, box1_size/2 -190/2 + 20))
        screen.blit(Up, (overlay_w/2 -62/2 - 5 , 10))

        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Down, (overlay_w/2 -115/2 + 6 , height -40))

        screen.blit(PivotL, (box3_size/2 - 190/2 - 15, height/2 -289/2))
        screen.blit(PRight, (overlay_w-box4_size/2 -284/2, height/2 + 170))

        screen.blit(PivotR, (overlay_w-box4_size/2 -190/2, height/2 -289/2))
        screen.blit(PLeft, (box3_size/2 - 255/2, height/2 +170))

        pygame.draw.rect(screen,(0,255,0), (overlay_w + 5 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(BaseHand,(overlay_w + (width-overlay_w)/2 - 228/2, height-170))

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

        X_sim, Y_sim = pygame.mouse.get_pos()

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
        #SimulateInput()
        #Calculate gaze position if gaze_pos_surf is not None
    
        if not keys[pygame.K_UP]:
            transparent_circle_smooth = pygame.Surface((40, 40), pygame.SRCALPHA)
            pygame.draw.circle(transparent_circle_smooth, (0, 255, 0, 128), (20, 20), 20)  # Semi-transparent green
            #screen.blit(transparent_circle_smooth, (x_smooth - 20, y_smooth - 20))

            transparent_circle_pixel = pygame.Surface((40, 40), pygame.SRCALPHA)
            pygame.draw.circle(transparent_circle_pixel, (255, 0, 0, 128), (20, 20), 20)  # Semi-transparent red
            screen.blit(transparent_circle_pixel, (X_sim - 20, Y_sim - 20))

            if box3_size < X_sim < overlay_w-box4_size and Y_sim < box1_size:
                box = 1
                Color1 = (0,255,0,128)
            elif box3_size < X_sim < overlay_w-box4_size and Y_sim > height-box2_size:
                box = 2
                Color2 = (0,255,0,128)
            elif box1_size < Y_sim < height-box2_size and X_sim < box3_size:
                box = 3
                Color3 = (0,255,0,128)
            elif box1_size < Y_sim < height-box2_size and X_sim > overlay_w-box4_size:
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

        
        
        
        DrawMenu(Menu)
        
        if keys[pygame.K_1]:
            Menu = 0
        
        if keys[pygame.K_2]:
            Menu = 1
        
        if keys[pygame.K_3]:
            Menu = 2
        
        if keys[pygame.K_4]:
            Menu = 3
        
        if keys[pygame.K_5]:
            Menu = 4


        VisInput()

        if keys[pygame.K_ESCAPE]:
            running = False

        if keys[pygame.K_UP]:
            pygame.draw.rect(screen, (0,255,0), (actuation), 3)
            locked_box = box
        else:
            locked_box = None

        if locked_box == 0:
            pass
           
        if locked_box == 1:
            #pygame.draw.rect(screen, (0,255,0), (box1_act), 5)
            Color1 = (255,0,0,128)
        if locked_box == 2:
            #pygame.draw.rect(screen,(0,255,0), (box2_act), 5)
            Color2 = (255,0,0,128)
        if locked_box == 3:
            #pygame.draw.rect(screen,(0,255,0), (box3_act), 5)
            Color3 = (255,0,0,128)
        if locked_box == 4:
            #pygame.draw.rect(screen,(0,255,0), (box4_act), 5)
            Color4 = (255,0,0,128)
        
        pygame.display.flip()



except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
