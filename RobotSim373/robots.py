from RobotSim373 import Robot,Box,Disk,Storage,connect
from math import cos,sin,radians

class TurtleBot(Robot):
    
    def __init__(self,*args,R=0.5,x=10,y=10,angle=0,**kwargs):
        
        super().__init__(*args)
        
        robot=self
        
        R=.5
        r=R/5
        disk_center=Disk(robot,x,y,radius=R,angle=angle,name='center')

        disks=[]
        for angle in range(0,360,30):
            disk=Disk(robot,
                              x+(R+1.1*r)*cos(radians(angle)),
                              y+(R+1.1*r)*sin(radians(angle)),
                     angle=angle,radius=r,
                     name='disk %d' % angle)
            disks.append(disk)

        connect(disk_center,disks,'weld')

        robot.disks=disks
        robot.angles=list(range(0,360,30))
        robot.distances=[-1]*len(disks)

        robot.S=Storage()
        robot.next_time=-1     
        
    def read_distances(self):
        return [disk.read_distance() for disk in self.disks]
        