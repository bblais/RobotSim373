#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('pylab', 'inline')


# In[2]:


from RobotSim373 import *


# In[3]:


def build(robot):
    box1=Box(robot,3,4.5,name='right')
    box2=Box(robot,3,6.5,name='left')
    
    connect(box1,box2,'weld')

    disk1=Disk(robot,2,5.5,name='center')
    
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
        
def act_forward_turn_example(t,robot):
    
    if robot['center'].x>12:
        robot['left'].F=-0.2
        robot['right'].F=-0.25
    else:
        robot['left'].F=0.4
        robot['right'].F=0.45
    
    if t>70:
        robot['left'].F=0
        robot['right'].F=0
        
        


# In[ ]:




