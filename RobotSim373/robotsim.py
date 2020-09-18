from numpy import sin,cos,degrees,radians,ndarray
from matplotlib.pyplot import imread
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

    def __init__(self,width=24,height=None,image=None):

        self.world = b2.world(gravity=(0, 0), doSleep=True)

        if not image is None:
            if isinstance(image,str):                
                self.im=imread(image)
            elif isinstance(image,ndarray):
                self.im=image
            else:
                raise ValueError

            height=width*self.im.shape[0]//self.im.shape[1]
        else:
            self.im=None
        
        if height is None:
            height=width


        self.height = height
        self.width = width

        if self.im is None:
            self.pixel_height=height
            self.pixel_width=width
        else:
            self.pixel_height=self.im.shape[0]
            self.pixel_width=self.im.shape[1]




        self.boundary = self.world.CreateStaticBody(position=(0, 0),
                    userData={'name':'walls'})
        self.boundary.CreateEdgeChain([(0, self.height),
                                (self.width, self.height),
                                (self.width, 0),
                                (0,0),
                                (0, self.height)]
                                )

        
        self.t=0.0

        self.robot=None
        
        self.objects=[]
        
    def to_pixels(self,x,y=None):

        if y is None:
            x,y=x

        W,H=self.width,self.height
        pW,pH=self.pixel_width,self.pixel_height

        px=int(x/W*pW)
        py=int((H-y)/H*pH)

        return int(px),int(py)
    
    def to_real(self,px,py=None):

        if py is None:
            px,py=px

        x=px*self.width/self.pixel_width
        y=(self.pixel_height-1-py)*self.height/self.pixel_height

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
        
        self._color='g'
        self.objects=[]
        
        self.message=None
        self.log=[]

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
    
    def __init__(self,parent,x,y,angle=0,width=1,height=1,name=None,
                angularDamping=0.2,linearDamping=0.2,
                restitution=0.2,friction=0.1,density=0.4):
        self.joints=[]
        self.motors=[]

        if isinstance(parent,Environment):
            self.env=parent
            self.parent=parent
        else:
            self.env=parent.env
            self.parent=parent

        self.width=width
        self.height=height
        
        self.body = self.env.world.CreateDynamicBody(position=(x, y), 
                                                    angle=0,
                                                     angularDamping=angularDamping,
                                                     linearDamping=linearDamping,
                                                  userData={'name':name})

        self.body.angle=radians(angle)
        self.fixture=self.body.CreatePolygonFixture(box=(self.width/2,self.height/2), 
                                                        density=density, friction=friction,
                                                        restitution=restitution,userData={'name':name})
            
        self.F=0
        self.F_angle=0
        self.τ=0


        self.color='b'
        if name is None:
            self.name='Box %d' % len(self.parent.objects)
        else:
            self.name=name
            
        self.parent+=self

    def read_color(self):

        if self.env.im is None:
            return [999,999,999]

        px,py=self.env.to_pixels(self.position.x,self.position.y)
        return self.env.im[py,px,:]


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
            F_hat=vec_mag_ang(1,degrees(self.body.angle)+self.F_angle)
            F_position=Vector(self.body.position)
            self.body.ApplyForce(self.F*F_hat, F_position, True)

        if self.τ:
            self.body.ApplyTorque(self.τ,True)


    def plot_orientation(self):
        from matplotlib.pyplot import plot
        x1,y1=self.position
        x2,y2=self.width/2*cos(radians(self.angle))+x1,self.width/2*sin(radians(self.angle))+y1
        plot([x1,x2],[y1,y2],'r-',lw=1)

        x2,y2=self.F/2*cos(radians(self.angle+self.F_angle))+x1,self.F/2*sin(radians(self.angle+self.F_angle))+y1
        plot([x1,x2],[y1,y2],'c-',lw=1)


class Disk(object):
    
    def __init__(self,parent,x,y,angle=0,radius=0.5,name=None,
                angularDamping=0.2,linearDamping=0.2,
                restitution=0.2,friction=0.1,density=0.4):
        self.joints=[]
        self.motors=[]

        if isinstance(parent,Environment):
            self.env=parent
            self.parent=parent
        else:
            self.env=parent.env
            self.parent=parent

        self.radius=radius
        
        self.body = self.env.world.CreateDynamicBody(position=(x, y), 
                                            angle=0,angularDamping=angularDamping,
                                                     linearDamping=linearDamping,
                                                  userData={'name':name})

        self.body.angle=radians(angle)
        
        self.fixture=self.body.CreateCircleFixture(radius=self.radius,
                                                   friction=friction, 
                                                   density=density,
                                                  restitution=restitution,
                                                  userData={'name':name})
            
        self.F=0
        self.F_angle=0
        self.τ=0

        self.color='b'
        if name is None:
            self.name='Circle %d' % len(self.parent.objects)
        else:
            self.name=name
        
        self.parent+=self

        

    def patch(self):
        from matplotlib.patches import Circle,Rectangle        
        return Circle((self.x,self.y),
                               self.radius)

    def plot_orientation(self):
        from matplotlib.pyplot import plot
        x1,y1=self.position
        x2,y2=self.radius*cos(radians(self.angle))+x1,self.radius*sin(radians(self.angle))+y1
        plot([x1,x2],[y1,y2],'r-',lw=1)

        x2,y2=self.F/2*cos(radians(self.angle+self.F_angle))+x1,self.F/2*sin(radians(self.angle+self.F_angle))+y1
        plot([x1,x2],[y1,y2],'c-',lw=1)

    def read_color(self):

        if self.env.im is None:
            return []

        px,py=self.env.to_pixels(self.position.x,self.position.y)
        return self.env.im[py,px,:]

        
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
            F_hat=vec_mag_ang(1,degrees(self.body.angle)+self.F_angle)
            F_position=Vector(self.body.position)
            self.body.ApplyForce(self.F*F_hat, F_position, True)
                        
        if self.τ:
            self.body.ApplyTorque(self.τ,True)



def connect(obj1,obj2,connection_type,**kwargs):

    if isinstance(obj1,list):
        for o1 in obj1:
            connect(o1,obj2,connection_type,**kwargs)
        return

    if isinstance(obj2,list):
        for o2 in obj2:
            connect(obj1,o2,connection_type,**kwargs)

        return


    if connection_type=='distance':

        joint = obj1.env.world.CreateDistanceJoint(
                            bodyA=obj1.body,
                            bodyB=obj2.body,
                            anchorA=obj1.body.position,
                            anchorB=obj2.body.position,
                            dampingRatio=10.0,**kwargs)
    elif connection_type=='weld':
        joint = obj1.env.world.CreateWeldJoint(
                            bodyA=obj1.body,
                            bodyB=obj2.body,
                            anchor= (
                                (obj1.body.position.x+obj2.body.position.x)/2,
                                (obj1.body.position.y+obj2.body.position.y)/2,
                            ),**kwargs)

    elif connection_type=='motor':
        offset=(obj1.body.position-obj2.body.position)/2
        joint = obj1.env.world.CreateRevoluteJoint(
            bodyA=obj1.body, 
            bodyB=obj2.body, 
            localAnchorA=-offset,
            localAnchorB=offset,
            maxMotorTorque=10000,
            **kwargs)    

        obj1.motors.append(joint)
        obj2.motors.append(joint)
    else:
        raise NotImplementedError
        
    obj1.joints.append(joint)
    obj2.joints.append(joint)
                            


def run_sim(env,act,total_time,dt=1/60,dt_display=1,
            figure_width=10,
            plot_orientation=True):
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
                plt.figure(figsize=(figure_width,figure_width*env.height//env.width))

                if not env.im is None:
                    plt.imshow(env.im,
                    interpolation=None,
                            extent=(0,env.width,0,env.height))

                patches,colors = zip(*[(b.patch(),b.color) for b in env.objects+robot.objects])
                p = PatchCollection(patches, 
                                    facecolors=colors,
                                cmap = matplotlib.cm.jet, 
                                alpha = 0.8)
                plt.gca().add_collection(p) 
                
                if plot_orientation:
                    for obj in robot.objects:
                        obj.plot_orientation()

                plt.axis('equal')
                plt.axis([0,env.width,0,env.height])
                if env.robot.message is None:
                    plt.title('%.2f' % env.t)
                else:
                    plt.title('%.2f Message: %s' % (env.t,
                                str(env.robot.message)))
                plt.show()


        except KeyboardInterrupt:
            stop=True    


    