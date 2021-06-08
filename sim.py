import pybullet
import pybullet_utils.bullet_client as bc

import pybullet_data
import numpy as np
import time
import yaml
import pickle
# import tf
from scipy.spatial.transform import Rotation as R
import time

from pointcloud.pointcloud import PointCloud

class PyBulletSim:
    """
    PyBulletSim: Implements panda hand movement
    """
    def __init__(self, sim_id, p, obj_name, use_random_objects=False, object_shapes=None, gui=False, isGravity=False):
        
        self.sim_id = sim_id
        self.p=p

        # load environment
        # if gui:
        #     self.p.connect(pybullet.GUI)
        # else:
        #     # self.p.connect(pybullet.DIRECT)
        #     self.p = bc.BulletClient(connection_mode=pybullet.DIRECT)
        #     print("Initialized Sim {}, p: {}".format(self.sim_id, self.p))
        # # self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # # self._plane_id = self.p.loadURDF("plane.urdf")
        if isGravity:
            self.p.setGravity(0, 0, -9.8)
        else:
            self.p.setGravity(0, 0, 0)
        self.gripper_id = None


        
        # loading objects
        self.objects_containers = yaml.safe_load(open('configs/objects.yml'))

        # define robot joints
        self.rb_revolute_id = 0
        self.rb_prismatic_id = 1
        self.lf_id = 2
        self.rf_id = 3
        self.gripper_dir = "assets/urdf_files/hand_modified.urdf"        

        # lists used for scaling
        self.obj_id = None
        self.obj_name = obj_name

        return None

    def process(self, grasp, score):
        # BELOW
        start_time = time.time()
        self.execute_grasp(grasp)

        # check weather there is a collision between gripper and object
        grasp_outcome = 0
        collision_container = self.p.getContactPoints(self.gripper_id, self.obj_id)
        if len(collision_container) > 0:
            grasp_outcome = 1

        self.p.disconnect()
        print(time.time()-start_time, ' seconds to finish for ', self.sim_id)

        return self.sim_id, grasp_outcome, score


    def generate_pc(self, obj_name=None, save_dir='temp/pc.npy'):
        pc = PointCloud()
        self.obj_id = self.load_object(self.obj_name, mass=0.00)
        self.step_simulation(100)
        pc.setCamera(cameraDistance=0.6, cameraYaw=40, cameraPitch=-25, cameraTargetPosition=[0,0.1,0])
        a = pc.generatePointCloud( verbose=True)
        pc.save(save_dir)
        #self.p.disconnect()


    def execute_grasp(self, grasp, p_end=[0,0,0]):
        
        # parse grasp
        trans = grasp[:3,3]
        #q = tf.transformations.quaternion_from_matrix(grasp)
        q = R.from_matrix(grasp[:3,:3]).as_quat()
        
        # transformation between grasp and down looking gripper 
        p_start = trans
        q_start = q
        p_end = p_end
        q_end = [0,1,0,0]
        p_trans,q_trans = self.p.multiplyTransforms(p_start,q_start,p_end,q_end)

        # convert object to new transform
        p_obj = [0,0,0]
        q_obj = [0,0,0,1]
        p_inv, q_inv = self.p.invertTransform(p_trans,q_trans)
        p_new, q_new = self.p.multiplyTransforms( p_inv, q_inv, p_obj,q_obj)


        # load gripper
        self.load_gripper(q=q_end, gripper_start_position=p_end)
        self.open_gripper()

        # upload object
        self.obj_id = self.load_object(self.obj_name, basePosition=p_new, baseOrientation=q_new, mass=0.01)
        
        # close gripper
        self.step_simulation(1000)
        self.close_gripper(force=0.1)
        self.step_simulation(1000)


        self.gripper_shake()

        # todo:
        # check wheather the grasp is successful
        

    def load_object(self, name=0, basePosition=[0,0,0], baseOrientation=[0,0,0,1], mass=0.05):
        fdir = self.objects_containers[name]['path']
        return self.load_object_to_scene(fdir, basePosition=basePosition, baseOrientation=baseOrientation, mass=mass)



    def step_simulation(self, num_steps=500):
        for i in range(int(num_steps)):
            self.p.stepSimulation()
            time.sleep(1e-3)

    def rb_translate_z(self, height=0.1, num_steps=500, verbose=False):
        if self.gripper_id is not None:
            # Constraints
            self.p.setJointMotorControlArray(
                self.gripper_id, [self.rb_prismatic_id], self.p.POSITION_CONTROL,
                targetPositions = [height],
                targetVelocities = [0.001],
                positionGains=0.01525*np.ones(1),
                forces=[0.1]
            )
        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                self.p.stepSimulation()
                if verbose:
                    print('joint ', 1, self.p.getJointState(self.gripper_id, 1  )[0])
                time.sleep(1e-3)


    def rb_rotate_y(self, angle=np.pi/2, num_steps=500, verbose=False):
        if self.gripper_id is not None:
            # Constraints
            self.p.setJointMotorControlArray(
                self.gripper_id, [self.rb_revolute_id], self.p.POSITION_CONTROL,
                targetPositions = [angle],
                targetVelocities = [0.01],
                positionGains=0.01*np.ones(1),
                forces=[0.1]
            )
        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                self.p.stepSimulation()
                if verbose:
                    print('joint ', 0, self.p.getJointState(self.gripper_id, 0  )[0])
                time.sleep(1e-3)



    def load_object_to_scene(self, cad_path, cad_scale=1, basePosition=[0.0,0.0,0.0], baseOrientation=[0,0,0,1], mass=0.05):
        # TODO
        object_body_id = None
        object_body_id = self.p.createCollisionShape(self.p.GEOM_MESH,
                                                fileName=cad_path,
                                                meshScale=[float(cad_scale), float(cad_scale), float(cad_scale)])
        if object_body_id is not None:
            object_body_id = self.p.createMultiBody(mass, object_body_id,
                                               basePosition=basePosition,
                                               baseOrientation=baseOrientation
                                               )
            return object_body_id
        else:
            print("Failed to load object")
            exit()



    # load panda gripper
    #TODO
    def load_gripper(self, gripper_start_position= [0,0,0], q=[0,0,0,1]):#, euler_angles=[0,0,0]):
        urdf_path = self.gripper_dir 
        
        # correct the gripper
        x,y,z = 0,0,1
        angle=np.pi/2
        q2= [x*np.sin(angle/2),y*np.sin(angle/2),z*np.sin(angle/2), np.cos(angle/2)]
        p3, q = self.p.multiplyTransforms(gripper_start_position,q, gripper_start_position, q2)

        if self.gripper_id is not None:
            print("Gripper already loaded")
            self.p.resetBasePositionAndOrientation(self.gripper_id, gripper_start_position, q)            
            return

        #print("Loading gripper")
        self.gripper_id = self.p.loadURDF(urdf_path, gripper_start_position, q, useFixedBase=1)
        # self.gripper_id_list.append(self.gripper_id)
        for i in range(self.p.getNumJoints(self.gripper_id)):
            self.p.changeDynamics(self.gripper_id, i, lateralFriction=1.0, spinningFriction=1.0,
                             rollingFriction=0.0001, frictionAnchor=True)


    def open_gripper(self, verbose=False):
        self.move_gripper(verbose=verbose)

    def close_gripper(self, force=0.1, verbose=False):
        self.move_gripper(position=0.0, force=force, verbose=verbose)

    def move_gripper(self, position=0.04, speed=0.000001, force=0.01, num_steps=1000, verbose=False):
        if self.gripper_id is not None:
             self.p.setJointMotorControlArray(
                self.gripper_id, [self.lf_id, self.rf_id], self.p.POSITION_CONTROL,
                targetPositions = [position, position],
                targetVelocities = [speed, speed],
                positionGains=np.ones(2),
                forces=[force, force]
                )

        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                self.p.stepSimulation()

                if verbose:
                    for i in [self.lf_id, self.rf_id]:
                        print('joint ', i, self.p.getJointState(self.gripper_id, i)[0])
                time.sleep(1e-3)

    def gripper_shake(self):
      
        self.rb_rotate_y(angle=np.pi/4, num_steps=250)
        self.rb_rotate_y(angle=-np.pi/4, num_steps=500)
        self.rb_rotate_y(angle=np.pi/4, num_steps=500)
        self.rb_rotate_y(angle=-np.pi/4, num_steps=500)
        self.rb_rotate_y(angle=np.pi/4, num_steps=500)
        self.rb_rotate_y(angle=-np.pi/4, num_steps=500)
        self.rb_rotate_y(angle=0, num_steps=250)

        self.rb_translate_z(height=0.05, num_steps=250)
        self.rb_translate_z(height=-0.05, num_steps=500)
        self.rb_translate_z(height=0.05, num_steps=500)
        self.rb_translate_z(height=-0.05, num_steps=500)
        self.rb_translate_z(height=0.05, num_steps=500)
        self.rb_translate_z(height=-0.05, num_steps=500)
        self.rb_translate_z(height=0, num_steps=250)  
        return


    def check_grasp_success(self):
        # TODO
        return
        #return self.p.getJointState(self._gripper_body_id, 1)[0] < 0.834 - 0.001




    def reset_scene(self):
        # TODO: cleans the scene from object and opens gripper in a predifined pose
        return
