from numpy import sin,cos,degrees,radians
from Box2D import b2,b2PolygonShape
from Box2D import b2Vec2 as Vector

b2p=30.0 # factor to convert coords
p2b=1/b2p

def Coords(Vector):  # native coord will be b2

    @property
    def b2(self):
        return self

    @property
    def pyglet(self):
        v2=self.copy()
        v2=v2*b2b
        v2[1]=window_height-v2[1]
        return v2

# In[2]:
def hat(vec):
    v2=vec.copy()
    v2.Normalize()
    return v2

def vec_mag_ang(mag,ang):
    from Box2D import b2Vec2 as Vector
    from math import cos,sin,radians
    x=mag*cos(radians(ang))
    y=mag*sin(radians(ang))
    return Vector(x,y)


# all units in real units
class Environment(object):

    def __init__(self,window_height=24,window_width=48):
        self.world = b2.world(gravity=(0, 0), doSleep=True)
        self.height = window_height
        self.width = window_width

        self.boundary = self.world.CreateStaticBody(position=(0, 0))
        self.boundary.CreateEdgeChain([(0, self.height),
                                (self.width, self.height),
                                (self.width, 0),
                                (0,0),
                                (0, self.height)]
                                )
        self.t=0.0

        self.robot=None
        
        self.objects=[]
        
    def __iadd__(self,other):
        self.objects.append(other)
        return self
    
    def update(self,dt):
        
        if self.robot:
            self.robot.update(dt)
            
        for obj in self.objects:
            obj.update(dt)
            
        self.world.Step(dt, 6, 2)
        self.world.ClearForces()
        self.t+=dt
            

class Robot(object):
    
    def __init__(self,env):
        self.env=env
        self.env.robot=self
        self.world=self.env.world
        
        self._color='g'
        self.objects=[]
        
    def update(self,dt):
        for obj in self.objects:
            obj.update(dt)
        
        
    @property
    def forces(self):
        return [obj.F for obj in self.objects]
    
    @forces.setter
    def forces(self,value):
        
        for v,obj in zip(value,self.objects):
            if not v is None:
                obj.F=v
        
        return [obj.F for obj in self.objects]
    
    
    def __getitem__(self,key):
        if isinstance(key,int):
            return self.objects[key]
        else:
            found=[]
            for obj in self.objects:
                if obj.name==key:
                    found.append(obj)
                    
            if not found:
                raise KeyError(f"Can't find key '{key}'")
                
            if len(found)>1:
                raise KeyError(f"Found more than one '{key}'")
            
            return found[0]
        
        
    @property
    def color(self):
        return self._color
    
    @color.setter
    def color(self,value):
        self._color=value
        for obj in self.objects:
            obj.color=value
    
    def __iadd__(self,other):
        self.objects.append(other)
        other.color=self.color
        
        return self


class Box(object):
    
    def __init__(self,env,x,y,angle=0,width=1,height=1,name=None,
                angularDamping=0.2,linearDamping=0.2,
                restitution=0.2,friction=0.1,density=0.4):
        self.joints=[]
        self.env=env
        self.width=width
        self.height=height
        
        self.body = self.env.world.CreateDynamicBody(position=(x, y), 
                                                    angle=0,
                                                     angularDamping=angularDamping,
                                                     linearDamping=linearDamping)

        self.body.angle=radians(angle)
        self.fixture=self.body.CreatePolygonFixture(box=(self.width/2,self.height/2), 
                                                        density=density, friction=friction,
                                                        restitution=restitution,)
            
        self.F=0
        self.color='b'
        if name is None:
            self.name='Box %d' % len(env.objects)
        else:
            self.name=name
            
        env+=self

    @property
    def corner_position(self):
        
#         pos=self.position
#         return pos[0]-self.width/2,pos[1]-self.height/2

        
        R=self.body.transform
        pos=R*self.fixture.shape.vertices[0]
        return pos
    
    def patch(self):
        from matplotlib.patches import Circle,Rectangle        
        return Rectangle((self.corner_x,self.corner_y),
                               self.width,self.height,
                               self.angle)        
    
    @property
    def corner_x(self):
        return self.corner_position[0]
    @property
    def corner_y(self):
        return self.corner_position[1]
    
    @property
    def position(self):
        return self.body.position
    
    @property
    def x(self):
        return self.position[0]
    @property
    def y(self):
        return self.position[1]
    
    @property
    def angle(self):
        return self.body.angle*180/3.14159
    
    def update(self,dt):
        if self.F:
            F_hat=vec_mag_ang(1,degrees(self.body.angle))
            F_position=Vector(self.body.position)
            self.body.ApplyForce(self.F*F_hat, F_position, True)
                    

class Disk(object):
    
    def __init__(self,env,x,y,angle=0,radius=0.5,name=None,
                angularDamping=0.2,linearDamping=0.2,
                restitution=0.2,friction=0.1,density=0.4):
        self.joints=[]
        self.env=env
        self.radius=radius
        
        self.body = self.env.world.CreateDynamicBody(position=(x, y), 
                                            angle=0,angularDamping=angularDamping,
                                                     linearDamping=linearDamping)

        self.body.angle=radians(angle)
        
        self.fixture=self.body.CreateCircleFixture(radius=self.radius,
                                                   friction=friction, 
                                                   density=density,
                                                  restitution=restitution,
                                                  )
            
        self.F=0
        self.color='b'
        if name is None:
            self.name='Circle %d' % len(env.objects)
        else:
            self.name=name
        
        env+=self
        

    def patch(self):
        from matplotlib.patches import Circle,Rectangle        
        return Circle((self.x,self.y),
                               self.radius)        
        
    @property
    def position(self):
        return self.body.position
    
    @property
    def x(self):
        return self.position[0]
    @property
    def y(self):
        return self.position[1]
    
    @property
    def angle(self):
        return self.body.angle*180/3.14159
    
    def update(self,dt):
        if self.F:
            F_hat=vec_mag_ang(1,degrees(self.body.angle))
            F_position=Vector(self.body.position)
            self.body.ApplyForce(self.F*F_hat, F_position, True)
                        


def connect(obj1,obj2,connection_type):
    if connection_type=='distance':

        joint = obj1.env.world.CreateDistanceJoint(
                            bodyA=obj1.body,
                            bodyB=obj2.body,
                            anchorA=obj1.body.position,
                            anchorB=obj2.body.position,
                            dampingRatio=10.0)
    elif connection_type=='weld':
        joint = obj1.env.world.CreateWeldJoint(
                            bodyA=obj1.body,
                            bodyB=obj2.body,
                            anchor= (
                                (obj1.body.position.x+obj2.body.position.x)/2,
                                (obj1.body.position.y+obj2.body.position.y)/2,
                            ),)
    
    else:
        raise NotImplementedError
        
    obj1.joints.append(joint)
    obj2.joints.append(joint)
                            


def run_sim(env,act,total_time,dt=1/60,dt_display=1):
    from IPython.display import clear_output
    import matplotlib
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle,Rectangle
    from matplotlib.collections import PatchCollection 
    
    
    env.t=0
    robot=env.robot

    stop=False
    next_display_t=-1
    while not stop:
        try:

            act(env.t,robot)

            env.update(dt)

            env.t+=dt

            if env.t>total_time:
                stop=True
                next_display_t=env.t-1


            if env.t>=next_display_t:
                next_display_t=env.t+dt_display

                clear_output(wait=True)
                plt.figure(figsize=(10,10*env.height//env.width))


                patches,colors = zip(*[(b.patch(),b.color) for b in env.objects+robot.objects])
                p = PatchCollection(patches, 
                                    facecolors=colors,
                                cmap = matplotlib.cm.jet, 
                                alpha = 0.8)
                plt.gca().add_collection(p) 
                plt.axis('equal')
                plt.axis([0,env.width,0,env.height])
                plt.title('%.2f' % env.t)
                plt.show()


        except KeyboardInterrupt:
            stop=True    


    