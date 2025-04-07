
import sys
sys.path.insert(1, '/its/home/drs25/Documents/GitHub/poppy-humanoid/hardware/URDF/robots')#
sys.path.insert(1, r'C:\Users\dexte\Documents\GitHub\poppy-humanoid\hardware\URDF\robots')
#path="C:/Users/dexte/Documents/GitHub/poppy-humanoid/hardware/URDF/robots/"
path="/its/home/drs25/Documents/GitHub/poppy-humanoid/hardware/URDF/robots/"
import pybullet as p
import pybullet_data
import time
import numpy as np
from poppyControl import *
import uuid
def demo(variable,history={}):
    return 0
class environment:
    def __init__(self,show=False,record=False,filename="",friction=0.5):
        self.show=show
        if show: p.connect(p.GUI)
        else: p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Ensure GUI is enabled
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  # Hide Explorer
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)  # Hide RGB view
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  # Hide Depth view
        self.friction=friction
        self.robot_id=None
        self.plane_id=None
        self.quad=None
        self.record=record
        self.filename=filename
        self.recording=0
        self.history={}
        if self.show:
            self.x_slider = p.addUserDebugParameter("dt", -5, 5, 0.1)
    def take_agent_snapshot(self,p, agent_id, alpha=0.1, width=640, height=480):
        # Make all objects except the agent transparent
        num_bodies = p.getNumBodies()
        for i in range(num_bodies):
            body_id = p.getBodyUniqueId(i)
            if body_id != agent_id:
                visual_shapes = p.getVisualShapeData(body_id)
                for visual in visual_shapes:
                    p.changeVisualShape(body_id, visual[1], rgbaColor=[1, 1, 1, alpha])  # Set transparency
        # Capture snapshot from the current camera view
        _, _, img_arr, _, _ = p.getCameraImage(width, height)
        # Convert the image array to a NumPy array
        return np.array(img_arr, dtype=np.uint8)
    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF('plane.urdf')
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Ensure GUI is enabled
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  # Hide Explorer
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)  # Hide RGB view
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  # Hide Depth view
        p.changeDynamics(self.plane_id, -1, lateralFriction=self.friction)
        p.setPhysicsEngineParameter(enableConeFriction=0)
        p.changeDynamics(self.plane_id, -1, lateralFriction=self.friction)
        initial_position = [0, 0, 0.45]  # x=1, y=2, z=0.5
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation (Euler angles to quaternion)
        flags = p.URDF_USE_SELF_COLLISION

        self.robot_id = p.loadURDF(path+"Poppy_Humanoid.URDF", initial_position, initial_orientation,flags=flags)
        p.changeDynamics(self.robot_id, -1, lateralFriction=self.friction)
        self.quad=Poppy(p,self.robot_id,self.plane_id)

        #self.quad.neutral=[-30, 0, 40, -30, 50, -10, 0, 10, 20, 30, -30, 50]
        self.quad.reset()
        for i in range(500):
            p.stepSimulation()
            p.setTimeStep(1./240.)
        if self.record:
            self.video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, self.filename)
            self.recording=1
        #if self.show:
            #self.x_slider = p.addUserDebugParameter("dt", -2, 2, 0.1)
    """def step(self,delay=0,T=1,dt=1):
        t=0
        while t<T:
            p.stepSimulation()
            if delay: time.sleep(1./240.)
            else: p.setTimeStep(1./240.)
            t+=dt"""
    def runTrial(self,agent,generations,delay=False,fitness=demo,photos=-1):
        history={}
        history['positions']=[]
        history['orientations']=[]
        history['motors']=[]
        history['accumalitive_reward']=[]
        history['self_collisions']=[]
        history['feet']=[]
        self.reset()
        a=[]
        photos_l=[]
        for i in range(generations*10):
            pos=self.step(agent,0,delay=delay)
            if photos>-1 and i%photos==0:
                print("snap")
                photos_l.append(self.take_agent_snapshot(p,self.robot_id))
            pos[[2,5,8,11]]=180-pos[[1,4,7,10]]
            basePos, baseOrn = p.getBasePositionAndOrientation(self.robot_id) # Get model position
            history['positions'].append(basePos)
            history['orientations'].append(baseOrn[0:3])
            history['motors'].append(pos)
            history['accumalitive_reward'].append(fitness(self.quad,history=history))
            history['self_collisions'].append(self.quad.get_self_collision_count())
            history['feet'].append(self.quad.getFeet())
            p.resetDebugVisualizerCamera( cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model
            if self.quad.hasFallen():
                break
            if self.quad.hasFallen():
                break
            a.append(pos)
        history['positions']=np.array(history['positions'])
        history['orientations']=np.array(history['orientations'])
        history['motors']=np.array(history['motors'])
        history['accumalitive_reward']=np.array(history['accumalitive_reward'])
        history['self_collisions']=np.array(history['self_collisions'])
        history['feet']=np.array(history['feet'])
        filename = str(uuid.uuid4())
        #np.save("/its/home/drs25/Documents/GitHub/Quadruped/Code/data_collect_proj/trials_all/"+str(filename),history)
        return fitness(self.quad,history=history),history,photos_l
    def step(self,agent,action,delay=False,gen=0,step=10):
        if self.show:
            agent.dt=p.readUserDebugParameter(self.x_slider)
        motor_positions=agent.get_positions(np.array(self.quad.motors))
        self.quad.setPositions(motor_positions)
        for k in range(step): #update simulation
            p.stepSimulation()
            if delay: time.sleep(1./240.)
            else: p.setTimeStep(1./240.)
            basePos, baseOrn = p.getBasePositionAndOrientation(self.robot_id) # Get model position
            p.resetDebugVisualizerCamera( cameraDistance=0.8, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model
            if self.quad.hasFallen():
                
                break
        return motor_positions
    def stop(self):
        if self.recording and self.record:
            p.stopStateLogging(self.video_log_id)
            self.recording=0
    def close(self):
        p.disconnect()

if __name__=="__main__":
    env=environment(1)
    env.reset()
    sys.path.insert(1, 'C:/Users/dexte/Documents/GitHub/Quadruped/Code')
    sys.path.insert(1, '/its/home/drs25/Documents/GitHub/Quadruped/Code')
    from CPG import *
    class newCPG(CTRNNQuadruped):
        def get_positions(self,inputs,motors=None):
            degrees=np.degrees(self.step(imu_feedback=0, velocity_feedback=0))/2
            degrees=np.clip(degrees,0,180)
            return degrees
        def step(self, imu_feedback, velocity_feedback):
            """Update the CTRNN for one timestep."""
            #compute neural activations (discrete update of CTRNN)
            net_input = self.weights @ self.outputs + self.biases
            self.activations += self.dt / self.tau * (-self.activations + net_input)
            self.outputs = self.sigmoid(self.activations)  #apply activation function
            #add oscillatory gait modulation
            self.phases += self.dt * self.omega
            oscillation = np.sin(self.phases)
            #compute motor commands (combining CTRNN output and oscillations)
            motor_commands = self.outputs#np.concatenate([self.outputs[0:3],self.outputs[0:3],self.outputs[0:3],self.outputs[0:3]]) + 0.5 * oscillation
            #apply IMU feedback for balance correction (modify hip joints)
            imu_correction = self.Kp_imu * imu_feedback  # Pitch correction
            motor_commands[::3] += imu_correction  # Adjust hips
            #apply velocity feedback for adaptive stride length (modify knees)
            velocity_correction = self.Kp_vel * velocity_feedback
            motor_commands[1::3] += velocity_correction  # Adjust knee motors
            return np.clip(motor_commands, 0, 1)  # Return motor positions (normalized)
    cpg=newCPG(num_n=25)
    print("commence trial...")
    #env.runTrial(cpg,500,1)
    time.sleep(3)
    print("move")
    action=np.zeros((25))
    action[0]=100
    env.step(cpg,action,1)
    print("trial done")
    time.sleep(5)
    env.close()