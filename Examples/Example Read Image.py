#!/usr/bin/env python
# coding: utf-8

# In[5]:


get_ipython().run_line_magic('pylab', 'inline')


# In[6]:


from RobotSim373 import *


# Each object in a robot has a color sensor, for the pixel below that part (center).

# In[3]:


def build(robot):
    box1=Box(robot,3,4.5,name='right')  # location given, width=height=1.0 default
    box2=Box(robot,3,6.5,name='left')

    connect(box1,box2,'weld')

    disk1=Disk(robot,2,5.5,name='center')  # radius = 0.5 default

    connect(disk1,box1,'distance')
    connect(disk1,box2,'distance')

    return robot


# In[4]:


def act_forward_backward_example(t,robot):

    if t<10:
        robot['left'].F=0.4
        robot['right'].F=0.4
    elif t<60:
        robot['left'].F=-0.4
        robot['right'].F=-0.4
    else:
        robot['left'].F=0
        robot['right'].F=0
        
    robot.message=str(robot['left'].read_color())+str(robot['right'].read_color())

env=Environment(image='background.png')  # size of the environment
robot=Robot(env)

robot=build(robot)

# put a bunch of blocks
for y in arange(1,20,0.5):
    Box(env,10,y,width=0.2,height=0.2,angle=30,density=0.001)

run_sim(env,act_forward_backward_example,
        total_time=80,  # seconds
        dt=1/60,
        dt_display=.1,  # make this larger for a faster display
       )


# In[ ]:




