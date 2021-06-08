# from __future__ import division
# from os import link
import sim
import pybullet_utils.bullet_client as bc
import pybullet as p
from multiprocessing import Process, Pipe
from joblib import Parallel, delayed
import pickle
import yaml

N_JOBS = 11 # number of threads available in PC
WORKS_PER_THREAD = 10 # make it small to overcome overhead on each thread




if __name__ == "__main__":

	# load info about all objects
	#objects_containers = yaml.safe_load(open('configs/objects.yml'))

	#objs = objects_containers.keys()
	#for obj_name in objects_containers.keys():
	obj_name = 'red_bowl'
	print(f'Processing {obj_name} ...')
	p.connect(p.GUI)
	#bc.BulletClient(connection_mode=pybullet.DIRECT)
	env = sim.PyBulletSim(0, p, obj_name)
	save_dir = f'assets/pcs/{obj_name}_trial1.npy'
	env.generate_pc(obj_name, save_dir)
	p.disconnect()