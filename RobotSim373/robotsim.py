from numpy import sin,cos,degrees,radians,ndarray,sqrt
from matplotlib.pyplot import imread
from Box2D import b2,b2PolygonShape,b2ContactListener
from Box2D import b2Vec2 as Vector

import matplotlib
matplotlib.rcParams["figure.figsize"]=10,8
matplotlib.rcParams["axes.titlesize"]=24
matplotlib.rcParams["axes.labelsize"]=20
matplotlib.rcParams["lines.linewidth"]=3
matplotlib.rcParams["lines.markersize"]=5
matplotlib.rcParams["xtick.labelsize"]=20
matplotlib.rcParams["ytick.labelsize"]=20
matplotlib.rcParams["axes.grid"]=True
matplotlib.rcParams["grid.linestyle"]='-'
matplotlib.rcParams["grid.color"]='cccccc'
matplotlib.rcParams["font.size"]=20
matplotlib.rcParams["font.family"]='sans-serif'
matplotlib.rcParams["legend.fontsize"]=20
matplotlib.rcParams["legend.frameon"]=False
matplotlib.rcParams["legend.numpoints"]=1
matplotlib.rcParams["legend.scatterpoints"]=1
matplotlib.rcParams["lines.solid_capstyle"]='round'
matplotlib.rcParams["text.color"]=[0.15]*3
matplotlib.rcParams["xtick.color"]=[0.15]*3
matplotlib.rcParams["ytick.color"]=[0.15]*3
matplotlib.rcParams["xtick.direction"]='out'
matplotlib.rcParams["ytick.direction"]='out'
matplotlib.rcParams["axes.axisbelow"]=True


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

from Box2D import b2RayCastCallback
class RayCastMultipleCallback(b2RayCastCallback):
    """This raycast collects multiple hits."""

    def __repr__(self):
        return 'Multiple hits'

    def __init__(self, **kwargs):
        b2RayCastCallback.__init__(self, **kwargs)
        self.fixture = None
        self.hit = False
        self.points = []
        self.normals = []
        self.fixtures=[]

    def ReportFixture(self, fixture, point, normal, fraction):
        from Box2D import b2Vec2
        
        self.hit = True
        self.fixture = fixture
        self.fixtures.append(fixture)
        self.points.append(b2Vec2(point))
        self.normals.append(b2Vec2(normal))
        return 1.0


class ContactListener(b2ContactListener):
    def __init__(self,env):
        b2ContactListener.__init__(self)
        self.env=env

    def BeginContact(self, contact):
        fixture_a = contact.fixtureA
        fixture_b = contact.fixtureB


        for obj in self.env.robot.objects:
            f=obj.fixture
            if f in [fixture_a,fixture_b]:
                obj.contact=True


    def EndContact(self, contact):
        pass

    def PreSolve(self, contact, oldManifold):
        pass
    def PostSolve(self, contact, impulse):
        pass




# all units in real units
class Environment(object):

    def __init__(self,width=24,height=None,image=None,gravity=0,
                    angularDamping=0.0,linearDamping=0.0,    # default values for objects
                    restitution=0.2,friction=0.1,density=0.4,plot_F_scale=1):

        self.angularDamping=angularDamping
        self.linearDamping=linearDamping
        self.restitution=restitution
        self.friction=friction
        self.density=density           
        self.plot_F_scale=plot_F_scale

        self.world = b2.world(gravity=(0, gravity), 
                            contactListener=ContactListener(self),
                            doSleep=True)

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
                    userData={'name':'walls',
                    'x':[0,self.width,self.width,0,0],
                    'y':[self.height,self.height,0,0,self.height]},
                    )
        self.boundary.CreateEdgeChain([(0, self.height),
                                (self.width, self.height),
                                (self.width, 0),
                                (0,0),
                                (0, self.height)]
                                )



        self.plot_orientation=True
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
    
    def __isub__(self,other):
        if other in self.objects:
            self.objects.remove(other)
            del other
            return self

        if isinstance(other,str):
            me=[_ for _ in self.objects if _.name==other]
            if me:
                for m in me:
                    self.objects.remove(m)
                    del m
                return self            

        return self


    def update(self,dt):
        
        if self.robot:
            self.robot.update(dt)
            
        for obj in self.objects:
            obj.update(dt)
            
        self.world.Step(dt, 6, 2)
        self.world.ClearForces()
        self.t+=dt
            

class FrictionEnvironment(Environment):

    def __init__(self,*args,damping=10,**kwargs):
        super().__init__(*args,**kwargs)
        if not 'linearDamping' in kwargs:
            self.linearDamping=damping

        if not 'angularDamping' in kwargs:
            self.angularDamping=damping
        

        if not 'plot_F_scale' in kwargs:
            self.plot_F_scale=0.1


   

class Controller(object):
    """
        forward_stop=StateMachine(
            (forward,'until_close'),
            ( (until_close,stop),'_end_simulation'),
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

        )    
    """
    def __init__(self,state_machine,verbose=False):
        self.state_machine=state_machine
        self.original_state_machine=state_machine
        self.current_state=self.state_machine.first_state
        self.start_time=0
        self.state_count=0
        self.monitor=None
        self.verbose=verbose

        
    def __call__(self,t,robot):
        return self.act(t,robot)
        
    def act(self,t,robot):
        if t==0.0:  # first time
            self.state_machine=self.original_state_machine
            self.current_state=self.state_machine.first_state
            self.start_time=0.0  
            self.state_count=0      
        
        try:
            current_actions=self.state_machine.states[self.current_state]['actions']
            next_state=self.state_machine.states[self.current_state]['next']
        except KeyError:
            print("The current state is: ",self.current_state)
            print("The available states are: ",self.state_machine.states)
            raise

            
        action=current_actions[self.state_count]
        
        if isinstance(action,StateMachine):  # skip to next StateMachine
            self.state_count=0
            self.state_machine=action
            self.current_state=self.state_machine.first_state
            current_actions=self.state_machine.states[self.current_state]['actions']
            next_state=self.state_machine.states[self.current_state]['next']
            self.start_time=t
            action=current_actions[self.state_count]
        
        
        
        value=action(t-self.start_time,robot)

        
        
        if value: # done with this action
            if self.verbose:
                print("value=",value)
            self.start_time=t
            
            if isinstance(value,str):
                self.current_state=value
                self.state_count=0

                if value=='_end_simulation':
                    if not self.monitor is None:
                        self.monitor(t,robot)

                    return True
                elif value !="_next_state":
                    self.state_count=0
                    
                    try:
                        self.state_machine.states[self.current_state]
                    except KeyError:
                        print("The current state is: ",self.current_state)
                        print("The available states are: ",self.state_machine.states)
                        raise
            elif isinstance(value,StateMachine):  # the action returned a state machine!
                self.state_machine=value
                self.current_state=self.state_machine.first_state
                self.state_count=0      
                self.start_time=t


            else:  # next action
                self.state_count+=1
                if self.state_count>=len(current_actions): # do next state
                    value="_next_state"
                    self.state_count=0
                else:
                    if self.verbose:
                        print(self.state_count,current_actions[self.state_count])



            if value=='_next_state':
                if self.verbose:
                    print(value,next_state)
                    
                self.current_state=next_state

                if isinstance(self.current_state,StateMachine):
                    self.state_machine=self.current_state
                    self.current_state=self.state_machine.first_state
                    self.state_count=0      

                
                if self.current_state=='_end_simulation':
                    return True

                try:
                    self.state_machine.states[self.current_state]
                except KeyError:
                    print("The current state is: ",self.current_state)
                    print("The available states are: ",self.state_machine.states)
                    raise


                self.start_time=t
            


        if not self.monitor is None:
            self.monitor(t,robot)
from collections import OrderedDict

class StateMachine(object):

    def __init__(self,*args,name=None):
        self.__name__=name
        
        self.functions={}
        self.states=OrderedDict()
        
        for i,arg in enumerate(args):
            if isinstance(arg[0],tuple) or isinstance(arg[0],list):
                key=arg[0][0].__name__
                _next=arg[1]
                if key=='_wait':
                    key=arg[0]._key

                self.states[key]={'actions':arg[0],"next":arg[1]}
                for func in arg[0]:
                    self.functions[func.__name__]=func
            else:
                func=arg[0]
                key=arg[0].__name__
                if key=='_wait':
                    key=arg[0]._key

                self.states[key]={'actions':[arg[0]],"next":arg[1]}
                self.functions[func.__name__]=func
        
        self.states["_end_simulation"]={'actions':None,"next":None}
        
        self.first_state=list(self.states.keys())[0]    

        for key in self.states:
            if key=="_end_simulation":
                continue
            next_state=self.states[key]['next']

            if isinstance(next_state,StateMachine):
                continue

            if next_state not in self.states:
                raise ValueError(f"Key '{next_state}' not found in {self.states.keys()}")
        
    

class Robot(object):
    
    def __init__(self,env):
        self.env=env
        self.env.robot=self
        
        self._color='g'
        self.objects=[]
        
        self.message=None
        self.storage=Storage()
        self.log=[]

    def offset(self,dx=None,dy=None):

        if not dx is None:
            for obj in self.objects:
                obj.body.position[0]+=dx
        if not dy is None:
            for obj in self.objects:
                obj.body.position[1]+=dy


    def update(self,dt):
        for obj in self.objects:
            obj.update(dt)
        
    @property
    def mass(self):
        return sum([obj.mass for obj in self.objects])

    def read_distances(self):
        return {obj.name:obj.read_distance() for obj in self.objects}


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

    def take_picture(self,filename='picture.png',show=False,**kwargs):
        from matplotlib.pyplot import savefig,close

        fig=display(self.env,show=show)
        fig.savefig(filename,**kwargs)

        if not show:
            close(fig)

    


class Box(object):
    
    def __init__(self,parent,x,y,angle=0,width=1,height=1,name=None,
                angularDamping=None,linearDamping=None,
                restitution=None,friction=None,density=None,color='b',
                plot_F_scale=None,
                ):


        self.joints=[]
        self.motors=[]

        if isinstance(parent,Environment):
            self.env=parent
            self.parent=parent
        else:
            self.env=parent.env
            self.parent=parent

        # default values from environment
        if angularDamping is None:
            angularDamping=self.env.angularDamping

        if linearDamping is None:
            linearDamping=self.env.linearDamping

        if restitution is None:
            restitution=self.env.restitution

        if friction is None:
            friction=self.env.friction

        if density is None:
            density=self.env.density

        if plot_F_scale is None:
            self.plot_F_scale=self.env.plot_F_scale


        self.width=width
        self.height=height
        
        self.body = self.env.world.CreateDynamicBody(position=(float(x), float(y)), 
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

        self.mass=self.body.mass


        self.color=color

        if name is None:
            self.name='Box %d' % len(self.parent.objects)
        else:
            self.name=name
            
        self.contact=False

        self.parent+=self

    def read_distance(self):
        callback=RayCastMultipleCallback()

        length = sqrt(self.env.width**2+self.env.height**2)
        point1 = self.position
        d = (length * cos(radians(self.angle)), length * sin(radians(self.angle)))
        point2 = point1 + d

        self.env.world.RayCast(callback, point1, point2)
        
        distances=[]
        for p in callback.points:
            x2,y2=p
            x1,y1=self.position
            distances.append(sqrt((x2-x1)**2+(y2-y1)**2))

        min_dist=1e500
        min_obj=None
        for f,d in zip(callback.fixtures,distances):
            obj=[_ for _ in self.env.objects if _.fixture == f]
            if f in self.env.boundary.fixtures:
                obj+=[self.env.boundary]

            assert ((len(obj)==0) or (len(obj)==1))

            if obj:
                if d<min_dist:
                    min_dist=d
                    min_obj=obj[0]

        assert not min_obj is None

        return min_dist
                    




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
        return Rectangle((self.corner_x,self._corner_y),
                               self.width,self.height,
                               self.angle)        
    
    @property
    def _corner_x(self):
        return self.corner_position[0]


    @property
    def _corner_y(self):
        return self.corner_position[1]
    
    
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
    def vx(self):
        return self.body.linearVelocity[0]

    @property
    def vy(self):
        return self.body.linearVelocity[1]

    @vy.setter
    def vy(self,value):
        self.body.linearVelocity[1]=value




    @property
    def vangle(self):
        return self.body.angularVelocity

    @property
    def angle(self):
        return self.body.angle*180/3.14159 % 360
    
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

        x2,y2=(self.F/2*self.plot_F_scale*cos(radians(self.angle+self.F_angle))+x1,
               self.F/2*self.plot_F_scale*sin(radians(self.angle+self.F_angle))+y1)

        plot([x1,x2],[y1,y2],'c-',lw=1)


class Disk(object):
    
    def __init__(self,parent,x,y,angle=0,radius=0.5,name=None,
                angularDamping=None,linearDamping=None,
                restitution=None,friction=None,
                density=None,color='b',
                plot_F_scale=None,
                ):

        self.joints=[]
        self.motors=[]


        if isinstance(parent,Environment):
            self.env=parent
            self.parent=parent
        else:
            self.env=parent.env
            self.parent=parent

        # default values from environment
        if angularDamping is None:
            angularDamping=self.env.angularDamping

        if linearDamping is None:
            linearDamping=self.env.linearDamping

        if restitution is None:
            restitution=self.env.restitution

        if friction is None:
            friction=self.env.friction

        if density is None:
            density=self.env.density

        if plot_F_scale is None:
            self.plot_F_scale=self.env.plot_F_scale

        self.radius=radius
        
        self.body = self.env.world.CreateDynamicBody(position=(float(x), float(y)), 
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
        self.mass=self.body.mass

        self.color=color
        if name is None:
            self.name='Circle %d' % len(self.parent.objects)
        else:
            self.name=name
        
        self.parent+=self
        self.contact=False

        

    def patch(self):
        from matplotlib.patches import Circle,Rectangle        
        return Circle((self.x,self.y),
                               self.radius)

    def read_distance(self):
        callback=RayCastMultipleCallback()

        length = sqrt(self.env.width**2+self.env.height**2)
        point1 = self.position
        d = (length * cos(radians(self.angle)), length * sin(radians(self.angle)))
        point2 = point1 + d

        self.env.world.RayCast(callback, point1, point2)
        
        distances=[]
        for p in callback.points:
            x2,y2=p
            x1,y1=self.position
            distances.append(sqrt((x2-x1)**2+(y2-y1)**2))

        min_dist=1e500
        min_obj=None
        for f,d in zip(callback.fixtures,distances):
            obj=[_ for _ in self.env.objects if _.fixture == f]
            if f in self.env.boundary.fixtures:
                obj+=[self.env.boundary]

            assert ((len(obj)==0) or (len(obj)==1))

            if obj:
                if d<min_dist:
                    min_dist=d
                    min_obj=obj[0]

        assert not min_obj is None

        return min_dist
                    
    def plot_orientation(self):
        from matplotlib.pyplot import plot
        x1,y1=self.position
        x2,y2=self.radius*cos(radians(self.angle))+x1,self.radius*sin(radians(self.angle))+y1
        plot([x1,x2],[y1,y2],'r-',lw=1)

        x2,y2=(self.F/2*self.plot_F_scale*cos(radians(self.angle+self.F_angle))+x1,
               self.F/2*self.plot_F_scale*sin(radians(self.angle+self.F_angle))+y1)
        plot([x1,x2],[y1,y2],'c-',lw=1)

    def read_color(self):

        if self.env.im is None:
            return [999.9,999.9,999.9]

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
    def vx(self):
        return self.body.linearVelocity[0]

    @property
    def vy(self):
        return self.body.linearVelocity[1]
    @vy.setter
    def vy(self,value):
        self.body.linearVelocity[1]=value
    
    @property
    def angle(self):
        return self.body.angle*180/3.14159 % 360
    

    @property
    def vangle(self):
        return self.body.angularVelocity

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
    elif connection_type=='revolve':
        joint = obj1.env.world.CreateRevoluteJoint(
                            bodyA=obj1.body,
                            bodyB=obj2.body,
                            anchor= obj2.body.position,
                            **kwargs)

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
                            

class Storage(object):
    def __init__(self):
        self.data=[]
    
    def __add__(self,other):
        s=Storage()
        s+=other
        return s
        
    def __iadd__(self,other):
        try:
            self.append(*other)
        except TypeError: # non-iterable
            self.append(other)

        return self
        
    def append(self,*args):
        if not self.data:
            for arg in args:
                self.data.append([arg])

        else:
            for d,a in zip(self.data,args):
                d.append(a)
       
    def arrays(self):
        from numpy import array

        tmp=[]
        for i in range(len(self.data)):
            tmp.append(array(self.data[i]))
            #self.data[i]=array(self.data[i])

        ret=tuple(tmp)
        if len(ret)==1:
            return ret[0]
        else:
            return ret

    def __array__(self):
        from numpy import vstack
        return vstack(self.arrays())

def until_xy(x,y,name='center'):
    
    def _until_xy(t,robot):
        try:
            xx=robot.original_x
        except AttributeError: 
            robot.original_x=robot[name].x
            robot.original_y=robot[name].y
            robot.original_distance2=(x-robot.original_x)**2+(y-robot.original_y)**2
            robot.last_distance2=robot.original_distance2
            
        robot.current_distance2=(x-robot[name].x)**2+(y-robot[name].y)**2

        if robot.current_distance2<10 and robot.current_distance2>robot.last_distance2:  # stop when getting farther away
            del robot.original_x, robot.original_y,robot.last_distance2
            return True
        
        robot.last_distance2=robot.current_distance2

        
    return _until_xy
            
def until_x(x,name='center'):
    
    def _until_x(t,robot):
        try:
            xx=robot.original_x
        except AttributeError: 
            robot.original_x=robot[name].x
            robot.original_dx_sign=(x-robot.original_x)>0
            
        robot.current_dx_sign=(x-robot[name].x)>0

        if robot.current_dx_sign!=robot.original_dx_sign:  # stop when crossing 
            del robot.original_x, robot.current_dx_sign,robot.original_dx_sign
            return True    
        
    return _until_x
            
def until_y(y,name='center'):
    
    def _until_y(t,robot):
        try:
            yy=robot.original_y
        except AttributeError: 
            robot.original_y=robot[name].y
            robot.original_dy_sign=(y-robot.original_y)>0
            
        robot.current_dy_sign=(y-robot[name].y)>0

        if robot.current_dy_sign!=robot.original_dy_sign:  # stop when crossing 
            del robot.original_y, robot.current_dy_sign,robot.original_dy_sign
            return True    
        
    return _until_y
                     
 

def wait(dt):

    def _wait(t,robot):
        if t<dt:
            return False
        else:
            return True

    f=_wait
    f._key='wait(%g)' % dt

    return _wait

def message2str(message):
    if isinstance(message,tuple):
        return " ".join([message2str(_) for _ in message])
    elif isinstance(message,float):
        return "{:.2f}".format(message)
    else:
        return str(message)



def display(env,robot=None,show=True):
    from IPython.display import clear_output
    import matplotlib
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle,Rectangle
    from matplotlib.collections import PatchCollection 


    clear_output(wait=True)
    fig=plt.figure(figsize=(env.figure_width,env.height*env.figure_width/env.width))
    env.fig=fig
    fig.clear()

    ax=fig.subplots()

    if not env.im is None:
        ax.imshow(env.im,
        interpolation=None,
                extent=(0,env.width,0,env.height))
    else:
        if env.show_boundary:
            bx,by=env.boundary.userData['x'],env.boundary.userData['y']
            plt.plot(bx,by,'r--',linewidth=1)


    if not robot is None:
        patches,colors = zip(*[(b.patch(),b.color) for b in env.objects+robot.objects])
    elif env.objects:
        patches,colors = zip(*[(b.patch(),b.color) for b in env.objects])
    else:
        patches,colors=None,None

    if patches:
        p = PatchCollection(patches, 
                            facecolors=colors,
                        cmap = matplotlib.cm.jet, 
                        alpha = 0.8)
        ax.add_collection(p) 
    
    if not robot is None and env.plot_orientation:
        for obj in robot.objects:
            obj.plot_orientation()

    plt.axis('equal')
    plt.axis([-0.1,env.width+.1,-.1,env.height+.1])


    if not robot is None:
        if env.robot.message is None:
            ax.set_title('%.2f' % env.t)
        else:
            ax.set_title('%.2f Message: %s' % (env.t,
                        message2str(env.robot.message)))

    if show:
        plt.show()
    return fig

def close(env,fig):
    from matplotlib import pyplot as plt
    plt.close(fig)

def run_sim(env,act,total_time,dt=1.0/60,dt_display=1,
            figure_width=10,
            plot_orientation=True,
            show_boundary=False):
    
    
    env.t=0
    env.dt=dt
    robot=env.robot

    env.plot_orientation=plot_orientation
    env.show_boundary=show_boundary

    stop=False
    next_display_t=-1
    env.figure_width=figure_width
    fig=None

    if isinstance(act,list):  # a list of act functions returning True to continue
        start_times=[None]*len(act)
    else:
        start_times=0.0

    count=0

    while not stop:
        try:
            if isinstance(act,list):  
                action_function=act[count]
                if start_times[count] is None:
                    start_times[count]=env.t                    
                value=action_function(env.t-start_times[count],robot)
                if value=='_end_simulation':
                    stop=True
                    next_display_t=env.t-1
                    break

                if value:  # next action
                    count+=1
                    if count>=len(act): # loop back to start of list
                        count=0
                        start_times=[None]*len(act)

            else:
                value=act(env.t,robot)
                if value:
                    stop=True
                    next_display_t=env.t-1

            env.update(dt)

            if env.t>total_time:
                stop=True
                next_display_t=env.t-1

            if not dt_display is None:
                if env.t>=next_display_t:
                    next_display_t=env.t+dt_display
                    fig=display(env,robot)

        except KeyboardInterrupt:
            fig=display(env,robot)
            stop=True    

    if not dt_display is None:
        close(env,fig)
    
# %%
