#!/usr/bin/env python3
import zmq
import msgpack
import numpy as np
import pygame.camera
import rospy
import std_msgs.msg
from std_msgs.msg import String as str_msg



pygame.init() #Initialize Pygame
width, height = 1790, 980 #Set parameters for vizualisation window
screen = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF) #Setup display
pygame.display.set_caption('Gaze vizu') #Name display window
clock = pygame.time.Clock() #Initialize a pygame clock for setting up frame rate


overlay_w = 1420 #Variable for setting where in the screen the gaze overlay stops, so how far to the right the overlay spans


#Assigning the April marker pngs to a varaible
April0 = 'April0.png'
April1 = 'April1.png'
April2 = 'April2.png'
April3 = 'April3.png'
 
lastbox = None  #History variable for box checking

pygame.camera.init() #Initialize so a camera feed can be shown with Pygame
cam = pygame.camera.Camera('/dev/video6') #Specify which camera is being used for the feed
cam.start() #Start the camera feed

#Load each April marker image
marker0 = pygame.image.load(April0)
marker1 = pygame.image.load(April1)
marker2 = pygame.image.load(April2)
marker3 = pygame.image.load(April3)

#Variables for each box, setting the widht of each box. The width of each box towards the middle. 
box1_size = 250 #Upper box
box2_size = 250 #Lower box
box3_size = 300 #Left box
box4_size = 300 #Right box


#Specifiying the parameters of each box: (Start X, Start Y, Widht, Height)
box1 = (box3_size, 0, overlay_w-box3_size-box4_size, box1_size) 
box2 = (box3_size, height-box2_size, overlay_w-box3_size-box4_size, box2_size)
box3 = (0, box1_size, box3_size, height-box1_size-box2_size)
box4 = (overlay_w-box4_size, box1_size, box4_size, height-box1_size-box2_size)

#Setting variables for each box color
Color1 = (171,194,196,128)
Color2 = (171,194,196,128)
Color3 = (171,194,196,128)
Color4 = (171,194,196,128)

#actuation = (overlay_w+15, height/4, width-overlay_w-30, height/2)
actuation_outline = (overlay_w+15, height/4, width-overlay_w-30, height/2)

#An empty box variable which is used for identifying which box the gaze is in. Gets assigned a new value when in box
box = None

#Setting up context
ctx = zmq.Context()

#Setting up request
req = zmq.Socket(ctx, zmq.REQ)

#Specifying Socket to connect to Pupil Capture
ip = 'localhost'
port = 50020

#Request a connection to Pupil Capture
req.connect(f'tcp://{ip}:{port}')

#Try to ask Pupil Capture for the port used for data streaming, the subsriber port
try:
   req.send_string('SUB_PORT') #Ask Pupil Capture for SUB_PORT
   sub_port = req.recv_string() #Assign the received message/port to sub_port
except zmq.ZMQError as e: #Error handling
    print(f'Error port: {e}') 
    running = False #Stop the program if no port is received

#Setting up subscribers to the data stream from Pupil Capture with the received port
surf_sub = ctx.socket(zmq.SUB) 
surf_sub.connect(f'tcp://{ip}:{sub_port}') #Connect to data stream
surf_sub.subscribe('surfaces.Surface 1') #Subsribe to the topic of Surface 1, our user interface

surf_sub.setsockopt(zmq.RCVTIMEO, 1000) #Set a timeout for the connection


running = True #Create a True statement for running the program
gaze_pos_surf = None #Create variable to assign gaze position on


Menu = 0 #Create variable for menu specification

#Function for receiving surface data from Pupil Capture
def recv_surf_data():
    global gaze_pos_surf #Create global variable
    try:
        topic, payload = surf_sub.recv_multipart() #Receive the data from Pupil Capture and split it into a topic and the message/payload
        message = msgpack.loads(payload) 
        #print(f"{topic}:{message}") #Print topic for debugging

        if 'gaze_on_surfaces' in message: #Checks if the message gaze_on_surfaces is in the received message
            gaze_on_surface = message['gaze_on_surfaces'] #Assigns the gaze_on_surface if message contains previous line

            for gaze in gaze_on_surface: #Going through the message gaze_on_surface
                if gaze['on_surf']: #If the "on_surf" is True in the recieved message then:
                    gaze_pos_surf = gaze['norm_pos'] #Assign the normalized gaze position to gaze_pos_surf

    except zmq.Again: #Error handling if message is empty. Pass.
        pass


#Function for drawing the gaze surface
def DrawGazeSurface():
    
    #Draws the gaze surface on the screen with specified boxes and lines.
    #The function performs the following drawing operations:
   
    #Creates a surface which can take an alpha/opacity value for each box. Alpha is 128 for each color
    Box1_transparent = pygame.Surface((box1[2], box1[3]), pygame.SRCALPHA)
    Box2_transparent = pygame.Surface((box2[2], box2[3]), pygame.SRCALPHA)
    Box3_transparent = pygame.Surface((box3[2], box3[3]), pygame.SRCALPHA)
    Box4_transparent = pygame.Surface((box4[2], box4[3]), pygame.SRCALPHA)

    #Draw each box on the surface with their specified color
    pygame.draw.rect(Box1_transparent, (Color1), (0, 0, box1[2], box1[3]), 0)
    pygame.draw.rect(Box2_transparent, (Color2), (0, 0, box2[2], box2[3]), 0)
    pygame.draw.rect(Box3_transparent, (Color3), (0, 0, box3[2], box3[3]), 0)
    pygame.draw.rect(Box4_transparent, (Color4), (0, 0, box4[2], box4[3]), 0)

    #Update the display with each box on the visualization window
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


#Draws the unused tongue tracker visualization
def DrawTongueViz():
    pygame.draw.rect(screen, (0, 15,8), (actuation_outline), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w+15, (height/4)*2.3), (width-16, (height/4)*2.3), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w + (width-overlay_w)/2, (height/4)*2.3), (overlay_w + (width-overlay_w)/2, ((height/4)*3)-1), 5)
    pygame.draw.line(screen, (0,0,0), (overlay_w+15+115.3, height/4), (overlay_w+15+115.3, (height/4)*2.3), 5)
    pygame.draw.line(screen, (0,0,0), (width-15-115.3, height/4), (width-15-115.3, (height/4)*2.3), 5)



#Draw the menus based on which menu we are in
def DrawMenu(Menu):
    
    #Draws the menu indicators which appear in the bottom right of the visualization window
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 1 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25 ,height-120, 50, 75), 3) #For menu 1
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 2 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3) #For menu 2
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 3 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3) #For menu 3
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 4 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3) #For menu 4
    pygame.draw.rect(screen, (0,15,8), (overlay_w + 5 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 25,height-120, 50, 75), 3) #For menu 5

    #Loads all the arrow images used for visualizing commands of each box
    ArrowUp = pygame.image.load('arrow_up.png')
    ArrowDown = pygame.image.load('arrow_down.png')
    ArrowLeft = pygame.image.load('arrow_left.png')
    ArrowRight = pygame.image.load('arrow_right.png')
    
    #Loads all the rotation arrows used to specify rotational directions
    R_Up = pygame.image.load('rotate_U.png')
    R_Down = pygame.image.load('rotate_D.png')
    R_Right = pygame.image.load('rotate_R.png')
    R_Left = pygame.image.load('rotate_L.png')

    #Loading text images to help the user distinguish each command
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

    #Loads the text images used to specify which menu is currently loaded
    Trans = pygame.image.load('Translational.png')
    Rot = pygame.image.load('Rotational.png')
    BaseHand = pygame.image.load('BaseHand.png')
    ForwardRoll = pygame.image.load('ForwardRoll.png')
    ForwardVertical = pygame.image.load('ForwardVert.png')

    #If statements which draws the corresponding images/arrows and text for each menu and their commands
    if Menu == 0:
        
        #Upper commmand
        screen.blit(ArrowUp, (overlay_w/2 -190/2, box1_size/2 -190/2 + 20))
        screen.blit(Up, (overlay_w/2 -62/2 - 5 , 10))

        #Lower command
        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Down, (overlay_w/2 -115/2 + 6 , height -40))

        #Left command
        screen.blit(ArrowLeft, (box3_size/2 -190/2, height/2 -190/2))
        screen.blit(Left, (box3_size/2 -115/2 +5 , height/2 + 110))

        #Right command
        screen.blit(ArrowRight, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        screen.blit(Right, (overlay_w-box4_size/2 -115/2 - 5 , height/2 + 110))

        #Creates a green box inside the corresponding menu visualizer in the bottom right and displays the menu name
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 1 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23 ,height-118, 46, 71), 0)
        screen.blit(Trans,(overlay_w + (width-overlay_w)/2 - 282/2, height-170))

    elif Menu == 1:
        
        #Upper commmand
        screen.blit(R_Up, (overlay_w/2 -190/2, box1_size/2 -190/2 + 30))
        screen.blit(PanU, (overlay_w/2 -145/2, 10))

        #Lower command
        screen.blit(R_Down, (overlay_w/2 -190/2, height-(box2_size/2) -190/2))
        screen.blit(PanD, (overlay_w/2 -211/2, height - 40))

        #Left command
        screen.blit(R_Right, (overlay_w-box4_size/2 -190/2 + 50, height/2 -190/2 - 10))
        screen.blit(PanR, (overlay_w-box4_size/2 -215/2, height/2 + 110))

        #Right command
        screen.blit(R_Left, (box3_size/2 -190/2, height/2 -190/2 - 10))
        screen.blit(PanL, (box3_size/2 -190/2, height/2 + 110))

        #Creates a green box inside the corresponding menu visualizer in the bottom right and displays the menu name
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 2 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(Rot,(overlay_w + (width-overlay_w)/2 - 306/2, height-170))

    elif Menu == 2:
        
        #Upper commmand
        screen.blit(ArrowUp, (overlay_w/2 -190/2 , box1_size/2 -190/2 + 20))
        screen.blit(Forward, (overlay_w/2 -150/2 -6 , 10 ))

        #Lower command
        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Back, (overlay_w/2 -93/2 + 6 , height -40))

        #Left command
        screen.blit(R_Left, (overlay_w-box4_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(Roll_L, (overlay_w-box4_size/2 -215/2 , height/2 + 110))
        
        #Right command
        screen.blit(R_Right, (box3_size/2 -190/2 , height/2 -190/2 -20))
        screen.blit(Roll_R, (box3_size/2 -239/2 , height/2 + 110))

        #Creates a green box inside the corresponding menu visualizer in the bottom right and displays the menu name
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 3 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(ForwardRoll,(overlay_w + (width-overlay_w)/2 - 266/2, height-170))
    elif Menu == 3:
        
        #Upper commmand
        screen.blit(ArrowUp, (overlay_w/2 -190/2 , box1_size/2 -190/2 + 20))
        screen.blit(Forward, (overlay_w/2 -150/2 -6 , 10 ))
        
        #Lower command
        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Back, (overlay_w/2 -93/2 + 6 , height -40))

        #Left command
        screen.blit(ArrowDown, (box3_size/2 -(190/2) - 15, height/2 -190/2))
        screen.blit(Down, (box3_size/2 -115/2 - 5 , height/2 + 110))

        #Right command
        screen.blit(ArrowUp, (overlay_w-box4_size/2 -190/2, height/2 -190/2))
        screen.blit(Up, (overlay_w-box4_size/2 -62/2 - 5 , height/2 + 110))
    
        #Creates a green box inside the corresponding menu visualizer in the bottom right and displays the menu name
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 4 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(ForwardVertical,(overlay_w + (width-overlay_w)/2 - 355/2, height-170))
    
    elif Menu == 4:

        #Upper commmand
        screen.blit(ArrowUp, (overlay_w/2 -190/2, box1_size/2 -190/2 + 20))
        screen.blit(Up, (overlay_w/2 -62/2 - 5 , 10))

        #Lower command
        screen.blit(ArrowDown, (overlay_w/2 -190/2 , height-(box2_size/2) -190/2 -20))
        screen.blit(Down, (overlay_w/2 -115/2 + 6 , height -40))

        #Left command
        screen.blit(PivotL, (box3_size/2 - 190/2 - 15, height/2 -289/2))
        screen.blit(PRight, (overlay_w-box4_size/2 -284/2, height/2 + 170))

        #Right command
        screen.blit(PivotR, (overlay_w-box4_size/2 -190/2, height/2 -289/2))
        screen.blit(PLeft, (box3_size/2 - 255/2, height/2 +170))
        
        #Creates a green box inside the corresponding menu visualizer in the bottom right and displays the menu name
        pygame.draw.rect(screen,(0,255,0), (overlay_w + 5 * (width-overlay_w)/5 - (1 * (width-overlay_w)/5)/2 - 23,height-118, 46, 71), 0)
        screen.blit(BaseHand,(overlay_w + (width-overlay_w)/2 - 228/2, height-170))

    else: #If its not within any of the specified menus, draw nothing
        pass
    
#Unused input visualizer
def VisInput():
    #if keys[pygame.K_KP4]: 
        pygame.draw.circle(screen, (0,255,0), (overlay_w+68, height/2.5), 20)
    
    #if keys[pygame.K_KP5]:
        pygame.draw.circle(screen, (0,255,0), (width-((width-overlay_w)/2), height/2.5), 20)

    #if keys[pygame.K_KP6]:
        pygame.draw.circle(screen, (0,255,0), (width-68, height/2.5), 20)

#Menu callback
def menu_calback(msg):
    global Menu
    Menu=msg.data

gaze_target_pubilsher=rospy.Publisher('gaze_comands', str_msg, queue_size=1)
sub_select_menu=rospy.Subscriber("/driver/menu/selected",std_msgs.msg.Int16,menu_calback)
rospy.init_node('gaze_node', anonymous=True)
rate = rospy.Rate(60)


try:
    while (not rospy.is_shutdown()):
        rate.sleep()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        

        #initialize keyboard input
        

        #set frame rate
        clock.tick(30)
        
        #draw background
        screen.fill((255,255,255))

        #Get camera feed
        img = cam.get_image()
        img = pygame.transform.scale(img, (overlay_w, height)) #Transform image to fit on overlay
        screen.blit(img, (0,0)) #Update display with camera feed

        #draw boxes
        DrawGazeSurface()
        
        #Draw markers for surface tracking
        screen.blit(marker0, (0,0))
        screen.blit(marker1, (0, height-207))
        screen.blit(marker2, (overlay_w-207, height-207))
        screen.blit(marker3, (overlay_w-204, 0))

        #Receive gaze data
        recv_surf_data()
        
        #Calculate gaze position if gaze_pos_surf is not None
        if gaze_pos_surf is not None:
            x_pixel = int(gaze_pos_surf[0] * overlay_w) #Calculating Pixel X position 
            y_pixel = height - int(gaze_pos_surf[1] * height) #Calculating Pixel Y position
          

            transparent_circle_pixel = pygame.Surface((40, 40), pygame.SRCALPHA) #Create surface for making gaze visulaization transparent
            pygame.draw.circle(transparent_circle_pixel, (255, 0, 0, 128), (20, 20), 20)  # Draw circle on surface
            screen.blit(transparent_circle_pixel, (x_pixel - 20, y_pixel - 20)) #Update display with gaze circle

            #If statements which specify which box the gaze is in, based on the box parameters
            if box3_size < x_pixel < overlay_w-box4_size and y_pixel < box1_size: #If gaze is within the x and y boundaries of box 1
                lastbox=box #update box history
                box = 1 
                Color1 = (0,255,0,128) #Make the box change color to a transparent green if gaze is within box
            elif box3_size < x_pixel < overlay_w-box4_size and y_pixel > height-box2_size: #If gzae is within the x and y boundaries of box 2
                lastbox=box
                box = 2
                Color2 = (0,255,0,128)#Make the box change color to a transparent green if gaze is within box
            elif box1_size < y_pixel < height-box2_size and x_pixel < box3_size: #If gzae is within the x and y boundaries of box 3
                lastbox=box
                box = 3
                Color3 = (0,255,0,128)#Make the box change color to a transparent green if gaze is within box
            elif box1_size < y_pixel < height-box2_size and x_pixel > overlay_w-box4_size: #If gzae is within the x and y boundaries of box 4
                lastbox=box
                box = 4
                Color4 = (0,255,0,128)#Make the box change color to a transparent green if gaze is within box
            else: #Else no box specified
                lastbox=box
                box = 0
                #Update the boxes back to its original color
                Color1 = (171,194,196,128) 
                Color2 = (171,194,196,128)
                Color3 = (171,194,196,128)  
                Color4 = (171,194,196,128)
            
            if lastbox!=box:
                msg_str="box:%s:" % box
                gaze_target_pubilsher.publish(msg_str)


        pygame.draw.line(screen, (0,15,8), (overlay_w,0+1),(overlay_w, height), 5)

        #Draw the unused tongue visualizer
        DrawTongueViz()

        
            
        
        #Draw menu based on specified menu
        DrawMenu(Menu)
        
        
        #VisInput()

        #Update display
        pygame.display.flip()



except KeyboardInterrupt:
    pass
finally:
    pygame.quit()