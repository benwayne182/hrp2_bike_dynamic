from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_tasks import generic6dReference, Task, GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture, MetaTaskKinePosture
from dynamic_graph import plug
from numpy import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application
from dynamic_graph.sot.tools import Oscillator, Seqplay
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces
import time

toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])

def degToRad(deg):
    rad=(deg*pi)/180
    return rad

def change6dPositionReference(task,feature,gain,position,selec=None,ingain=None,resetJacobian=True):
    M=generic6dReference(position)
    if selec!=None:
        if isinstance(selec,str):  feature.selec.value = selec
        else: feature.selec.value = toFlags(selec)
    feature.reference.value = matrixToTuple(M)
    if gain!=None:  setGain(gain,ingain)
    if 'resetJacobianDerivative' in task.__class__.__dict__.keys() and resetJacobian:
        task.resetJacobianDerivative()

class Hrp2Bike(Application):

    tracesRealTime = True

    initialPose = (
        # Free flyer
        0., 0., 0., 0., 0. , 0.,

        # Legs
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,

        # Chest and head
        0., 0., 0., 0.,

        # Arms
        0., 0., 0., 0., 0., 0., 0.1,
        0., 0.,  0., 0., 0., 0., 0.1,
        )

    def __init__(self,robot,hands=True):#, forceSeqplay=True):
        Application.__init__(self,robot)

        self.sot=self.solver.sot
        self.hands=hands
#        self.robot=robot
#        self.seq=Seqplay('seqplay')
        self.createTasks(robot)
        self.initTasks()
        self.initTaskGains()
        self.initSolver()
        self.initialStack()

    def printSolver(self): #show tasks in solver controling the robot
        print self.solver


    #----------TASKS-------------------------------
    #------------------CREATION--------------------

    def createTrunkTask (self, robot, taskName, ingain=1.0):
        task = Task (taskName)
        task.add (self.features['chest'].name)
        task.add (self.features['waist'].name)
        task.add (self.features['gaze'].name)
        gain = GainAdaptive(taskName+'gain')
        gain.setConstant(ingain)
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error)
        return (task, gain)

    def createTasks(self,robot):
        self.taskHalfSitting = MetaTaskKinePosture(self.robot.dynamic,'halfsitting')
        self.taskInitialPose = MetaTaskKinePosture(self.robot.dynamic,'initial-pose')
        self.taskGripper     = MetaTaskKinePosture(self.robot.dynamic,'gripper')

        (self.tasks['trunk'],self.gains['trunk'])= self.createTrunkTask(robot, 'trunk')

        self.taskRF          = self.tasks['left-ankle']
        self.taskLF          = self.tasks['right-ankle']
        self.taskCom         = self.tasks['com']
        self.taskRH          = self.tasks['right-wrist']
        self.taskLH          = self.tasks['left-wrist']
        self.taskTrunk       = self.tasks['trunk']
        self.taskPosture     = self.tasks['posture']
        self.taskBalance     = self.tasks['balance']
        self.taskWaist       = self.tasks['waist']

    def openGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperOpen,),lhand=(self.gripperOpen,))

    def closeGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperClose,),lhand=(self.gripperClose,))

    #------------------INIT-TASK------------------
    def initTasks(self):
        self.initTaskBalance()
        self.initTaskPosture()
        if self.hands:
            self.initTaskGripper()
        self.initTaskHalfSitting()
        self.initTaskInitialPose()


    def initTaskBalance(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111000'
        self.featureCom.selec.value = '111'

    def initTaskHalfSitting(self):
        self.taskHalfSitting.gotoq(None,\
                                    rleg =(self.robot.halfSitting[6:12]),   \
                                    lleg =(self.robot.halfSitting[12:18]),  \
                                    chest=(self.robot.halfSitting[18:20]),  \
                                    head =(self.robot.halfSitting[20:22]),  \
                                    rarm =(self.robot.halfSitting[22:28]),  \
                                    larm =(self.robot.halfSitting[29:35]))

    def initTaskInitialPose(self):
        self.taskInitialPose.gotoq(None,\
                                    rleg =(self.initialPose[6:12]),   \
                                    lleg =(self.initialPose[12:18]),  \
                                    chest=(self.initialPose[18:20]),  \
                                    head =(self.initialPose[20:22]),  \
                                    rarm =(self.initialPose[22:28]),  \
                                    larm =(self.initialPose[29:35]))

    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 12
        weight_head      = 0.3
        weight_arm       = 0.8

        weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        weight[9,9] = weight_knee
        weight[15,15] = weight_knee
        weight[19,19] = weight_chesttilt
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        mask = '1'*36
        # mask = 6*'0'+12*'0'+4*'1'+14*'0'
        # mask = '000000000000111100000000000000000000000000'
        # robot.dynamic.displaySignals ()
        # robot.dynamic.Jchest.value
        self.features['posture'].selec.value = mask

    def initTaskGripper(self):
        self.gripperOpen = degToRad(30)
        self.gripperClose = degToRad(3)
        self.openGripper()


    #------------------INIT-GAIN------------------
    def initTaskGains(self):
        self.taskHalfSitting.gain.setByPoint(2,0.2,0.01,0.8)
        self.taskGripper.gain.setConstant(3)
        self.taskInitialPose.gain.setByPoint(2,0.2,0.01,0.8)

    #----------SOLVER------------------------------
    def initSolver(self):
        plug(self.sot.control,self.robot.device.control)

    def push(self,task): #push task in solver 
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName not in toList(self.sot):
            self.sot.push(taskName)
        if taskName!="posture" and "posture" in toList(self.sot):
            self.sot.down("posture")

    def removeTasks(self,task) : #remove task from solver
            if isinstance(task,str): taskName=task
            elif "task" in task.__dict__:  taskName=task.task.name
            else: taskName=task.name
            if taskName in toList(self.sot):
                self.sot.remove(taskName)

    def printSolver(self): #show tasks in solver controlling the robot
        print self.solver

    def showTasks(self) : #show library of precomputed tasks
        self.tasks

    # --- TRACES -----------------------------------------------------------
    def withTraces(self):
        if self.tracesRealTime:
            self.robot.initializeTracer()
        else:
            self.robot.tracer = Tracer('trace')
            self.robot.device.after.addSignal('{0}.triger'.format(self.robot.tracer.name))
        self.robot.tracer.open('/tmp/','','.dat')
        self.robot.startTracer()

    def stopTraces(self):
        self.robot.stopTracer()

    def trace(self):
        self.robot.tracer.dump()

    #----------RUN---------------------------------
    def goInitialPose(self):
        self.sot.clear()
#        self.push(self.taskBalance)
        self.push(self.taskLF)
        self.push(self.taskRF)
        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)
        self.push(self.taskInitialPose)

    def initialStack(self):
        self.push(self.taskBalance)
        self.push(self.taskTrunk)
        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)

    def goHalfSitting(self):
        self.sot.clear()
        self.push(self.taskBalance)
        self.push(self.taskPosture)
        if self.hands:
            self.push(self.taskGripper)
        self.push(self.taskHalfSitting)

    #------------parameters of bike-------
    xg=0.0; zg=0.29 #center of crank gear
    R=0.17 #pedal-center of crank gear

    def goBikeSitting(self):
        self.sot.clear()
#        self.push(self.taskBalance)
#        self.push(self.tasks['chest'])
        change6dPositionReference(self.taskWaist,self.features['waist'],\
                                    self.gains['waist'],\
                                    (-0.22,0.0,0.58,0,0,0),'111111')
        self.push(self.taskWaist)
        if self.hands:
            self.push(self.taskGripper)
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    (0.25,-0.255,0.90,-pi/2,0,pi/2),'111111')
        self.push(self.taskRH)
        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     (0.25,0.255,0.90,pi/2,0,-pi/2),'111111')
        self.push(self.taskLH)
        change6dPositionReference(self.taskRF,self.features['right-ankle'],\
                                    self.gains['right-ankle'],\
                                    (0.0,-0.1125,0.12,0,0,degToRad(-8)),'111111')
        self.push(self.taskRF)
        change6dPositionReference(self.taskLF,self.features['left-ankle'],\
                                    self.gains['left-ankle'],\
                                    (0.0,0.1125,0.46,0,0,degToRad(8)),'111111')
        self.push(self.taskLF)
        self.push(self.taskPosture)

    def stopMove(self):
        self.rotation=False

    def circle(self,nbPoint=16,rotation=True): #nbPoint=number of point to dicretise the circle
        self.rotation=rotation
        self.stepCircle=(2*pi)/nbPoint
        circleL=[]
        circleR=[]
        for j in range(0,nbPoint):
            circleL.append(-(j*self.stepCircle)+(pi/2))
            circleR.append(-(j*self.stepCircle)+(3*pi/2))
        self.sot.clear()
        change6dPositionReference(self.taskWaist,self.features['waist'],\
                                    self.gains['waist'],\
                                    (-0.22,0.0,0.58,0,0,0),'111111')
        self.push(self.taskWaist)
        if self.hands:
            self.push(self.taskGripper)
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    (0.25,-0.255,0.90,-pi/2,0,pi/2),'111111')
        self.push(self.taskRH)
        change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     (0.25,0.255,0.90,pi/2,0,-pi/2),'111111')
        self.push(self.taskLH)
        #--------rotation--------
        self.push(self.taskRF)
        self.push(self.taskLF)
        self.push(self.taskPosture)
        #while self.rotation:
        for i in range(0,nbPoint):
            xpL=(cos(circleL[i])*self.R)+self.xg
            zpL=(sin(circleL[i])*self.R)+self.zg
            xpR=(cos(circleR[i])*self.R)+self.xg
            zpR=(sin(circleR[i])*self.R)+self.zg
            change6dPositionReference(self.taskRF,self.features['right-ankle'],\
                                        self.gains['right-ankle'],\
                                        (xpR,-0.1125,zpR,0,0,degToRad(-8)),'111111')
            change6dPositionReference(self.taskLF,self.features['left-ankle'],\
                                        self.gains['left-ankle'],\
                                        (xpL,0.1125,zpL,0,0,degToRad(8)),'111111')
            time.sleep(5)
    # --- SEQUENCER ---
    step=1
    def sequencer(self,stepSeq=None):
        if stepSeq!=None:
            self.step=stepSeq
        #-----initial position------
        if self.step==0:
            print "Step : ", self.step
            self.goInitialPose()
            print('Initial Pose')
            self.step+=1
        elif self.step==1:
            print "Step : ", self.step
            self.goBikeSitting()
            print('Bike Sitting')
            self.step+=1
        elif self.step==2:
            print "Step : ", self.step
            if self.hands:
                self.closeGripper()
            print('Close Gripper')
            self.step+=1
        #-----move start------
        elif self.step==3:
            print "Step : ", self.step
            self.circle()
            print('Start movement')
            self.step+=1
        #-----end of move-----
        #elif self.step==4:
            #print "Step : ", self.step
            #self.stopMove()
            #print('Stop movement')
           # self.step+=1
        elif self.step==4:#5:
            print "Step : ", self.step
            if self.hands:
                self.openGripper()
            print('Open Gripper')
            self.step+=1
        else:
            print "Step : ", self.step
            self.goHalfSitting()
            print('Half-Sitting')
            self.step+=1
    def __call__(self):
        self.sequencer()
    def __repr__(self): 
        self.sequencer()
        return str()
