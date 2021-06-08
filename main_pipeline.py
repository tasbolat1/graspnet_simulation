# from __future__ import division
# from os import link
import sim
import pybullet_utils.bullet_client as bc
import pybullet
from multiprocessing import Process, Pipe
from joblib import Parallel, delayed
import pickle
import yaml

N_JOBS = 11 # number of threads available in PC
WORKS_PER_THREAD = 10 # make it small to overcome overhead on each thread

def run_process(sim_id, grasp, score, obj_name):

	p = bc.BulletClient(connection_mode=pybullet.DIRECT)
	env = sim.PyBulletSim(sim_id, p, obj_name)
	return env.process(grasp, score)


def load_grasps(load_dir):
	all_data = pickle.load(open(load_dir, 'rb'))
	return  all_data['grasps'], all_data['scores']



if __name__ == "__main__":

	# load info about all objects
	objects_containers = yaml.load(open('configs/objects.yml'))
	print(objects_containers)


	# obj_name = 'mustard_bottle'

	# grasps, scores = load_grasps('temp/mustard.pkl')
	
	# # to check
	# grasps = grasps[:20]

	# print('There are ', len(grasps), ' grasps')

