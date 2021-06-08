import pybullet as p
import pybullet_data
import numpy as np
import time
import yaml
import pickle
import tf

from pointcloud.pointcloud import PointCloud

class PyBulletSim:
    """
    PyBulletSim: Implements two tote UR5 simulation environment with obstacles for grasping 
        and manipulation
    """
    def __init__(self, use_random_objects=False, object_shapes=None, gui=True, isGravity=False):
        # 3D workspace for tote 1
        self._workspace1_bounds = np.array([
            [0.38, 0.62],  # 3x2 rows: x,y,z cols: min,max
            [-0.22, 0.22],
            [0.00, 0.5]
        ])
        # 3D workspace for tote 2
        self._workspace2_bounds = np.copy(self._workspace1_bounds)
        self._workspace2_bounds[0, :] = - self._workspace2_bounds[0, ::-1]

        # load environment
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self._plane_id = p.loadURDF("plane.urdf")
        if isGravity:
            p.setGravity(0, 0, -9.8)
        else:
            p.setGravity(0, 0, 0)
        self.gripper_id = None


        
        # loading objects
        self.objects_containers = yaml.load(open('configs/objects.yml'))
        self.object_ids = []
        # for obj in self.objects_containers.keys():
        #     if obj == 'mustard_bottle':
        #         object_id = self.load_object(obj, basePosition=[0,0,0])
        #         self.object_ids.append(object_id)
        #         # self.step_simulation(30000)
        #         #p.removeBody(object_id)
        





        # for grasp in self.grasps:
        #     position = grasp[:3,3]
        #     orietation 
        #     print(position)
        #     self.load_gripper("/home/crslab/pybullet_tests/graspGripper2/assets/urdf_files/hand_modified.urdf", basePosition)
        #     exit()
        # loading gripper

        # define robot joints
        self.rb_revolute_id = 0
        self.rb_prismatic_id = 1
        self.lf_id = 2
        self.rf_id = 3
        self.gripper_dir = "assets/urdf_files/hand_modified.urdf"        
        # self.open_gripper()
        # self.close_gripper()
        # self.gripper_shake()

        #self.load_object('mustard_bottle')
        #self.generate_pc()
        

        # x,y,z = 0,0,1
        # angle=np.pi/2
        # p1 = [0,0,0]
        # p2 = [0,0,0]
        # q1 = [0,0,0,1]
        # q2= [x*np.sin(angle/2),y*np.sin(angle/2),z*np.sin(angle/2), np.cos(angle/2)]
        # self.load_gripper()
        # p3, q3 = p.multiplyTransforms(p1,q1, p2, q2)

        # print(q3)
        # p.resetBasePositionAndOrientation(self.gripper_id, p3, q3)   
        # self.open_gripper()
       


        all_data = pickle.load(open('temp/mustard.pkl', 'rb'))

        self.grasps = all_data['grasps']
        self.scores = all_data['scores']
        # self.point_cloud = all_data['point_cloud']
        # obj_trans = np.mean(self.point_cloud, axis=0)

        score = self.scores[0]#23
        grasp = self.grasps[0]#23
        # obj_id = self.execute_grasp(grasp, score)

        self.execute_grasp(grasp, score)

        trans = grasp[:3,3]
        q = tf.transformations.quaternion_from_matrix(grasp)
        # self.load_gripper(q=q, gripper_start_position=trans)
        # self.open_gripper()
        # print('GRASP SCORE: ', score)
        # self.load_object('mustard_bottle', mass=0.1)



        # q_orig = [0,0,0,1]
        # x,y,z=[0,1,0]
        # alpha=np.pi
        # q2 = [x*np.sin(alpha/2), y*np.sin(alpha/2), z*np.sin(alpha/2), np.cos(alpha/2)]
        # p3,q3 = p.multiplyTransforms([0,0,0],q_orig,[0,0,0],q2)

        # print(p3)
        # print(q3)

        

        # p_start = trans
        # q_start = q
        # p_end = [0,0,0]
        # q_end = [0,1,0,0]
        # p_trans,q_trans = p.multiplyTransforms(p_start,q_start,p_end,q_end)
        # print('trans: ', trans)
        # print('p_trans: ', p_trans)

        # p_obj = [0,0,0]
        # q_obj = [0,0,0,1]
        # #p_new, q_new = p.multiplyTransforms(p_trans,q_trans,p_obj,q_obj)
        # p_inv, q_inv = p.invertTransform(p_trans,q_trans)
        # p_new, q_new = p.multiplyTransforms( p_inv, q_inv, p_obj,q_obj)

        # print('p_new: ', p_new)
        
        # self.load_object('mustard_bottle',basePosition=p_new, baseOrientation=q_new, mass=0.1)
        
        # self.load_gripper(q=q_end, gripper_start_position=p_end)

        # self.step_simulation(1000)
        # self.close_gripper()
        # self.step_simulation(1000)



        # print('going up')

        # self.rb_translate_z(-0.5)


         

        # # self.open_gripper()
        # print('GRASP SCORE: ', score)

        # p1 = [0,0,0]
        # q1 = [0,0,0,1]
        # print(trans)
        # p3,q3 = p.multiplyTransforms(p1,q1,trans,q)

        # self.load_object('mustard_bottle',basePosition=p3, baseOrientation=q3, mass=0.1)

        # self.step_simulation(10000)

        # self.close_gripper()


        # # for i, (grasp, score) in enumerate( zip(self.grasps, self.scores) ):
        # #     print('i: ', i)
        # #     obj_id = self.execute_grasp(grasp, score)
        # #     self.step_simulation(1000)

        # #     p.removeBody(obj_id)
        # #     p.removeBody(self.gripper_id)
        # #     self.gripper_id=None


        # #     print('::::::', obj_id, self.gripper_id)
        # #     self.step_simulation(10)

        self.step_simulation(50000)
        p.disconnect()

        return None

    def generate_pc(self, save_dir='temp/pc.npy'):
        pc = PointCloud()
        #pc.stepX = 5
        #pc.stepY = 5
        pc.setCamera(cameraDistance=0.6, cameraYaw=40, cameraPitch=-25, cameraTargetPosition=[0,0.1,0])
        a = pc.generatePointCloud( verbose=True)
        pc.save(save_dir)


    def execute_grasp(self, grasp, score):
        
        # parse grasp
        trans = grasp[:3,3]
        q = tf.transformations.quaternion_from_matrix(grasp)
        
        # transformation between grasp and down looking gripper 
        p_start = trans
        q_start = q
        p_end = [0,0,0]
        q_end = [0,1,0,0]
        p_trans,q_trans = p.multiplyTransforms(p_start,q_start,p_end,q_end)

        # convert object to new transform
        p_obj = [0,0,0]
        q_obj = [0,0,0,1]
        p_inv, q_inv = p.invertTransform(p_trans,q_trans)
        p_new, q_new = p.multiplyTransforms( p_inv, q_inv, p_obj,q_obj)


        # load gripper
        self.load_gripper(q=q_end, gripper_start_position=p_end)
        self.open_gripper()

        # upload object
        self.load_object('mustard_bottle',basePosition=p_new, baseOrientation=q_new, mass=0.01)
        
        # close gripper
        self.step_simulation(1000)
        self.close_gripper(force=0.1)
        self.step_simulation(1000)

        #self.rb_translate_z(-0.5)
        print("{} shaking gripper".format(self.p))
        self.gripper_shake()
        print("{} done shaking gripper".format(self.p))

        

    # def execute_grasp(self, grasp, score):
    #     trans = grasp[:3,3]
    #     q = tf.transformations.quaternion_from_matrix(grasp)
    #     self.load_gripper(q=q, gripper_start_position=trans)
    #     self.open_gripper()
    #     print('GRASP SCORE: ', score)
    #     return self.load_object('mustard_bottle', mass=0.1)


    def load_object(self, name=0, basePosition=[0,0,0], baseOrientation=[0,0,0,1], mass=0.05):
        fdir = self.objects_containers[name]['path']
        print(fdir)
        return self.load_object_to_scene(fdir, basePosition=basePosition, baseOrientation=baseOrientation, mass=mass)



    def step_simulation(self, num_steps=500):
        for i in range(int(num_steps)):
            p.stepSimulation()
            time.sleep(1e-3)

    def rb_translate_z(self, height=0.1, num_steps=500, verbose=False):
        if self.gripper_id is not None:
            # Constraints
            p.setJointMotorControlArray(
                self.gripper_id, [self.rb_prismatic_id], p.POSITION_CONTROL,
                targetPositions = [height],
                targetVelocities = [0.001],
                positionGains=0.01525*np.ones(1),
                forces=[0.1]
            )
        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                p.stepSimulation()
                if verbose:
                    print('joint ', 1, p.getJointState(self.gripper_id, 1  )[0])
                time.sleep(1e-3)


    def rb_rotate_y(self, angle=np.pi/2, num_steps=500, verbose=False):
        if self.gripper_id is not None:
            # Constraints
            p.setJointMotorControlArray(
                self.gripper_id, [self.rb_revolute_id], p.POSITION_CONTROL,
                targetPositions = [angle],
                targetVelocities = [0.01],
                positionGains=0.01*np.ones(1),
                forces=[0.1]
            )
        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                p.stepSimulation()
                if verbose:
                    print('joint ', 0, p.getJointState(self.gripper_id, 0  )[0])
                time.sleep(1e-3)



    def load_object_to_scene(self, cad_path, cad_scale=1, basePosition=[0.0,0.0,0.0], baseOrientation=[0,0,0,1], mass=0.05):
        # TODO
        object_body_id = None
        object_body_id = p.createCollisionShape(p.GEOM_MESH,
                                                fileName=cad_path,
                                                meshScale=[float(cad_scale), float(cad_scale), float(cad_scale)])
        if object_body_id is not None:
            object_body_id = p.createMultiBody(mass, object_body_id,
                                               basePosition=basePosition,
                                               baseOrientation=baseOrientation
                                               )
            # self.reset_object()
            # self.random_reset_object(object_body_id)
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
        p3, q = p.multiplyTransforms(gripper_start_position,q, gripper_start_position, q2)

        if self.gripper_id is not None:
            print("Gripper already loaded")
            p.resetBasePositionAndOrientation(self.gripper_id, gripper_start_position, q)            
            return

        print("Loading gripper")
        self.gripper_id = p.loadURDF(urdf_path, gripper_start_position, q, useFixedBase=1)
        for i in range(p.getNumJoints(self.gripper_id)):
            p.changeDynamics(self.gripper_id, i, lateralFriction=1.0, spinningFriction=1.0,
                             rollingFriction=0.0001, frictionAnchor=True)


    def open_gripper(self, verbose=False):
        self.move_gripper(verbose=verbose)

    def close_gripper(self, force=0.1, verbose=False):
        self.move_gripper(position=0.0, force=force, verbose=verbose)

    def move_gripper(self, position=0.04, speed=0.000001, force=0.01, num_steps=1000, verbose=False):
        if self.gripper_id is not None:
             p.setJointMotorControlArray(
                self.gripper_id, [self.lf_id, self.rf_id], p.POSITION_CONTROL,
                targetPositions = [position, position],
                targetVelocities = [speed, speed],
                positionGains=np.ones(2),
                forces=[force, force]

                )

        for i in range(int(num_steps)):
            if self.gripper_id is not None:
                p.stepSimulation()

                if verbose:
                    for i in [self.lf_id, self.rf_id]:
                        print('joint ', i, p.getJointState(self.gripper_id, i)[0])
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
        #return p.getJointState(self._gripper_body_id, 1)[0] < 0.834 - 0.001














    # def execute_grasp(self, grasp_position, grasp_angle):
    #     """
    #         Execute grasp sequence
    #         @param: grasp_position: 3d position of place where the gripper jaws will be closed
    #         @param: grasp_angle: angle of gripper before executing grasp from positive x axis in radians 
    #     """
    #     # # Adjust grasp_position to account for end-effector length
    #     # grasp_position = grasp_position + self._tool_tip_to_ee_joint
    #     # gripper_orientation = p.getQuaternionFromEuler(
    #     #     [np.pi, 0, grasp_angle])
    #     # pre_grasp_position_over_bin = grasp_position+np.array([0, 0, 0.3])
    #     # pre_grasp_position_over_object = grasp_position+np.array([0, 0, 0.1])
    #     # post_grasp_position = grasp_position+np.array([0, 0, 0.3])
    #     # grasp_success = False
    #     # # ========= PART 2============
    #     # # Implement the following grasp sequence:
    #     # # 1. open gripper
    #     # # 2. Move gripper to pre_grasp_position_over_bin
    #     # # 3. Move gripper to pre_grasp_position_over_object
    #     # # 4. Move gripper to grasp_position
    #     # # 5. Close gripper
    #     # # 6. Move gripper to post_grasp_position
    #     # # 7. Move robot to robot_home_joint_config
    #     # # 8. Detect whether or not the object was grasped and return grasp_success
    #     # # ============================
    #     # self.open_gripper()
    #     # self.move_tool(grasp_position, gripper_orientation)
    #     # self.close_gripper()
    #     # self.move_tool(post_grasp_position, None)
    #     # self.robot_shake(post_grasp_position, grasp_angle)
    #     # self.robot_go_home(speed=0.03)
    #     # grasp_success = self.check_grasp_success()
    #     # time.sleep(1)
    #     grasp_success = 0

    #     return grasp_success









    def reset_scene(self):
        # TODO: cleans the scene from object and opens gripper in a predifined pose
        return



    def random_reset_object(self, object_body_id):
        # for object_body_id in self._objects_body_ids:
        random_position = np.random.random_sample((3))*(self._workspace1_bounds[:, 1]-(
            self._workspace1_bounds[:, 0]+0.1))+self._workspace1_bounds[:, 0]+0.1
        random_orientation = np.random.random_sample((3))*2*np.pi-np.pi
        p.resetBasePositionAndOrientation(
            object_body_id, random_position, p.getQuaternionFromEuler(random_orientation))
        self.step_simulation(500)
