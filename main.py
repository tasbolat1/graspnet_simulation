# from __future__ import division
# from os import link
import sim
import pybullet_utils.bullet_client as bc
import pybullet
from multiprocessing import Process, Pipe
from joblib import Parallel, delayed
import pickle

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

	#p = bc.BulletClient(connection_mode=pybullet.GUI)
	# env = sim.PyBulletSim(arg1, p)
	# env.process()
	obj_name = 'mustard_bottle'

	grasps, scores = load_grasps('temp/mustard.pkl')
	
	# to check
	grasps = grasps[:20]

	print('There are ', len(grasps), ' grasps')

	n_iterations = N_JOBS*WORKS_PER_THREAD
	if len(grasps) < n_iterations:
		n_iterations = len(grasps)

	results = []
	process_list = []
	count=0
	while len(grasps) > 0:
		for i in range(n_iterations):
			grasp = grasps.pop(0)
			score = scores.pop(0)
			process_list.append([count, grasp, score, obj_name])
			count+=1

		res = Parallel(n_jobs=N_JOBS)(delayed(run_process)(*zz) for zz in process_list)
		results.append(res)

	print('Done')
	print(results)





    

	# p.connect(p.GUI)
	# env = sim.PyBulletSim(0, p)
	# res = env.process()




	# n_jobs=4
	# parent_conn, child_conn = Pipe()
	# proc = Process(target=Worker, args=(0,2,child_conn))
	# proc.start()
	# print(parent_conn.recv())   # prints "[42, None, 'hello']"
	# proc.join()
	# big_list_tact = [[0,1],[1,1],[2,1]]
	# all_data = pickle.load(open('temp/mustard.pkl', 'rb'))
	# grasps = all_data['grasps']
	# scores = all_data['scores']
	# # big_list should contain a subset of the grasps


	# big_list_tact = [(grasp,score) for grasp,score in zip(grasps,scores)]
	# lst = range(0,10)

	# approx_sizes = len(lst)/n_jobs 

	# groups_cont = [lst[int(i*approx_sizes):int((i+1)*approx_sizes)] 
	#                for i in range(n_jobs)]
    


	# exit()
	# a= Parallel(n_jobs=n_jobs)(delayed(run_process)(*zz) for zz in big_list_tact)
	# print(a)

    # for sim_id in range(2):
    # 	p = bc.BulletClient(connection_mode=pybullet.DIRECT)
    # 	print('here')
    # 	env = sim.PyBulletSim(sim_id, p)


