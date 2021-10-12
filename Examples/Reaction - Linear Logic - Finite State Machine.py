#!/usr/bin/env python
# coding: utf-8

# In[ ]:


get_ipython().run_line_magic('pylab', 'inline')


# In[ ]:


from RobotSim373 import *


# In[ ]:


def build(robot):
    box1=Box(robot,3,9.5,name='right')  # location given, width=height=1.0 default
    box2=Box(robot,3,11.5,name='left')
    
    connect(box1,box2,'weld')

    disk1=Disk(robot,2,10.5,name='center')  # radius = 0.5 default
    
    connect(disk1,box1,'distance')
    connect(disk1,box2,'distance')
    
    


# In[ ]:


def act(t,robot):
    
    distance=robot['center'].read_distance()
    if distance>10:
        robot['left'].F=0.4
        robot['right'].F=0.4
    else:
        robot['left'].F=-0.4
        robot['right'].F=-0.4
    


# In[ ]:


env=Environment(24,24)  # size of the environment
robot=Robot(env)

build(robot)

# put a bunch of blocks
for y in arange(5,15,0.5):
    Box(env,20,y,width=0.4,height=0.4,density=0.01)

run_sim(env,act,
        total_time=80,  # seconds
        dt=1/60,
        dt_display=.5,  # make this larger for a faster display
       )


# In[ ]:


robot.read_distances()


# ## Do this then that

# In[ ]:


def act1(t,robot):
    robot.message="here 1 ",t
    
    if t>5:
        return True
    
def act2(t,robot):
    robot.message="here 2",t

    if t>5:
        return True
    
    
def act3(t,robot):
    robot.message="here 3",t
    
    if t>5:
        return True
    


# In[ ]:


env=Environment(24,24)  # size of the environment
robot=Robot(env)

build(robot)

# put a bunch of blocks
for y in arange(5,15,0.5):
    Box(env,20,y,width=0.4,height=0.4,density=0.01)

run_sim(env,[act1,act2,act3],
        total_time=80,  # seconds
        dt=1/60,
        dt_display=.5,  # make this larger for a faster display
       )


# In[ ]:


def forward(t,robot):
    robot['left'].F=0.4
    robot['right'].F=0.4
    return True


# In[ ]:


def backward(t,robot):
    robot['left'].F=-0.4
    robot['right'].F=-0.4
    return True


# In[ ]:


def until_close(t,robot):
    distance=robot['center'].read_distance()
    if distance<10:
        return True


# In[ ]:


def until_far(t,robot):
    distance=robot['center'].read_distance()
    if distance>10:
        return True


# In[ ]:


env=Environment(24,24)  # size of the environment
robot=Robot(env)

robot=build(robot)

# put a bunch of blocks
for y in arange(5,15,0.5):
    Box(env,20,y,width=0.4,height=0.4,density=0.01)

run_sim(env,[forward,until_close,backward,until_far],
        total_time=100,  # seconds
        dt=1/60,
        dt_display=.5,  # make this larger for a faster display
       )


# In[ ]:


def forward(t,robot):
    robot['left'].F=10
    robot['right'].F=10
    return True


# In[ ]:


def backward(t,robot):
    robot['left'].F=-10
    robot['right'].F=-10
    return True


# In[ ]:





# In[ ]:


env=FrictionEnvironment(24,24)  # size of the environment
robot=Robot(env)
build(robot)

# put a bunch of blocks
for y in arange(5,15,0.5):
    Box(env,20,y,width=0.4,height=0.4,density=0.01)

run_sim(env,[forward,until_close,backward,until_far],
        total_time=100,  # seconds
        dt=1/60,
        dt_display=.5,  # make this larger for a faster display
       )


# ## Finite State Machine

# In[ ]:


state_machine=StateMachine(
    (forward,'until_close'),
    (until_close,'backward'),
    (backward,'until_far'),
    (until_far,'forward'),
)


def monitor(t,robot):
    robot.message=robot.controller.current_state


# In[ ]:


env=FrictionEnvironment(24,24)  # size of the environment
robot=Robot(env)

build(robot)
robot.controller=Controller(state_machine)
robot.controller.monitor=monitor

# put a bunch of blocks
for y in arange(5,15,0.5):
    Box(env,20,y,width=0.4,height=0.4,density=0.01)

run_sim(env,robot.controller,
        total_time=100,  # seconds
        dt=1/60,
        dt_display=.5,  # make this larger for a faster display
       )


# ## A more complex example

# In[ ]:


def forward(t,robot):
    robot['left'].F=4
    robot['right'].F=4
    return True

def stop(t,robot):
    robot['left'].F=0
    robot['right'].F=0
    return True


def backward(t,robot):
    robot['left'].F=-4
    robot['right'].F=-4
    return True

def until_close(t,robot):
    distance=robot['center'].read_distance()
    if distance<6:
        return True
        
        
        
def look_left(t,robot):
    robot['center'].τ=.2

    if robot['center'].angle>45:
        robot['center'].τ=0
        return True
    
def save_distance_left(t,robot):
    robot['center'].left_distance=robot['center'].read_distance()
    return True

def look_right(t,robot):
    robot['center'].τ=-.2
    
    if robot['center'].angle>300 and robot['center'].angle<315:
        robot['center'].τ=0
        return True
    
def save_distance_right(t,robot):
    robot['center'].right_distance=robot['center'].read_distance()
    return True
    
    
def look_straight(t,robot):
    robot['center'].τ=.2
    
    if robot['center'].angle>0 and robot['center'].angle<10:
        robot['center'].τ=0
        return True
    
    
    
def choose_right_or_left(t,robot):
    
    L,R=robot['center'].left_distance,robot['center'].right_distance
    
    if L>1.1*R:
        return 'turn_left_45'
    elif R>1.1*L:
        return 'turn_right_45'
    else:
        return 'backward'
    
    
    
def delta_angle(a1,a2):
    return 180 - abs(abs(a1 - a2) - 180)


def turn_left_45(t,robot):
    
    robot['left'].F=-4
    robot['right'].F=4
    
    if delta_angle(robot['right'].angle,0)>45:
        return True
    
def turn_right_45(t,robot):
    robot['left'].F=4
    robot['right'].F=-4
    
    if delta_angle(robot['right'].angle,0)>45:
        return True
    
    
    
def until_closer(t,robot):
    distances=robot.read_distances()

    distance=min(distances['right'],distances['left'])
    if distance<2:
        return True
    
    
def until_far(t,robot):
    distances=robot.read_distances()
    if distances['center']>12:
        return True        


# In[ ]:


forward_stop=StateMachine(
    (forward,'until_closer'),
    ( (until_closer,stop),'_end_simulation'),
    name='forward_stop'
)

state_machine=StateMachine(
    (forward,'until_close'),
    ( (until_close,stop),'look_left'),
    ( (look_left,save_distance_left,wait(2)),'look_right'),
    ( (look_right,save_distance_right,wait(2)),'look_straight'),
    (look_straight,'choose_right_or_left'),
    (choose_right_or_left,'_end_simulation'),
    (turn_left_45,'forward_stop'),
    (turn_right_45,'forward_stop'),
    (forward_stop,'_end_simulation'),
    ( (backward,until_far),'_end_simulation'),
    
)


# In[ ]:


def monitor(t,robot):
    robot.message=robot.controller.current_state,robot['left'].read_distance(),robot['right'].read_distance(),robot['center'].read_distance(),robot['center'].angle


# In[ ]:


env=FrictionEnvironment(24,24)  # size of the environment
robot=Robot(env)

build(robot)
robot.controller=Controller(state_machine,verbose=False)
robot.controller.monitor=monitor

Box(env,15,15,width=0.4,height=10,density=0.01)

#Box(env,15,7,width=0.4,height=10,density=0.01)  # should go left

#Box(env,15,12,width=0.4,height=20,density=0.01)  # should back up and end


run_sim(env,robot.controller,
        total_time=100,  # seconds
        dt=1/60,
        dt_display=0.5,  # make this larger for a faster display
       )


# In[ ]:




