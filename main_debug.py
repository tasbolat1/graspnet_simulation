# from __future__ import division
# from os import link
import sim
import pybullet_utils.bullet_client as bc
import pybullet
from multiprocessing import Process, Pipe
from joblib import Parallel, delayed
import pickle

def run_process(arg1, arg2):

	p = bc.BulletClient(connection_mode=pybullet.GUI)
	env = sim.PyBulletSim(arg1, p)
	return env.process()


if __name__ == "__main__":
	n_jobs=4
	# parent_conn, child_conn = Pipe()
	# proc = Process(target=Worker, args=(0,2,child_conn))
	# proc.start()
	# print(parent_conn.recv())   # prints "[42, None, 'hello']"
	# proc.join()
	# big_list_tact = [[0,1],[1,1],[2,1]]
	all_data = pickle.load(open('temp/mustard.pkl', 'rb'))
	grasps = all_data['grasps']
	scores = all_data['scores']
	# big_list should contain a subset of the grasps


	big_list_tact = [(grasp,score) for grasp,score in zip(grasps,scores)]
	lst = range(0,10)

	approx_sizes = len(lst)/n_jobs 

	groups_cont = [lst[int(i*approx_sizes):int((i+1)*approx_sizes)] 
	               for i in range(n_jobs)]
    


	exit()
	a= Parallel(n_jobs=n_jobs)(delayed(run_process)(*zz) for zz in big_list_tact)
	print(a)

    # for sim_id in range(2):
    # 	p = bc.BulletClient(connection_mode=pybullet.DIRECT)
    # 	print('here')
    # 	env = sim.PyBulletSim(sim_id, p)


