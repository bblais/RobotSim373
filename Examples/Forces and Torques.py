#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('pylab', 'inline')


# In[2]:


from RobotSim373 import *


# In[3]:


def build(robot):

    width=2
    height=0.3

    
    Box(robot,
        x=10,
        y=10,
        width=width,
        height=height,
        name='bob')

    
    Box(robot,
        x=10,
        y=16,
        width=width,
        height=height,
        name='sally')
    
    
def act(t,robot):
    
    robot['bob'].F=.3  # force
    
    robot['sally'].τ=.3  # torque
    


# In[4]:


env=Environment(24,24) 
robot=Robot(env)

robot=build(robot)


for y in arange(1,20,5):
    Box(env,10,y,width=1,height=1,angle=30,density=0.001)

run_sim(env,act,
        total_time=80,  # seconds
        dt=1/60,
        dt_display=0.1,  # make this larger for a faster display
        plot_orientation=True,
        figure_width=8,
       )


# 1. How much force to move at a constant speed?
# 2. How much torque to rotate at constant speed?

# In[ ]:




