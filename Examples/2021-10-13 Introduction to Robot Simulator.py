#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('pylab', 'inline')


# In[2]:


from RobotSim373 import *


# In[21]:


def build(robot):
    box1=Box(robot,x=3,y=9.5,name="right")
    box2=Box(robot,x=3,y=11.5,name="left")    
    
    connect(box1,box2,"weld")
    
    disk1=Disk(robot,x=2,y=10.5,name="center")
    
    connect(disk1,box1,"distance")
    connect(disk1,box2,"distance")    


# In[30]:


def act(t,robot):
    
    distance=robot['center'].read_distance()
    
    if distance>10:
        robot['left'].F=0.1
        robot['right'].F=0.1   
    else:
        robot['left'].F=0.1
        robot['right'].F=0.1   
    
    robot.message=distance


# In[31]:


env=Environment(24,24)
robot=Robot(env)
build(robot)

Box(env,x=15,y=11,width=0.2,height=10)


run_sim(env,act,
        figure_width=6,
       total_time=80,
       dt_display=0.5,  # make this larger for a faster display
       )


# In[ ]:




