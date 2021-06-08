# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np 

# #tgtMesh = mesh.Mesh.from_file('/home/crslab/pybullet_tests/graspGripper2/assets/10_objects/004_sugar_box/google_16k/textured.obj')
pc = np.load('temp/pc.npy')


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# # print(pc)
# #ax.add_collection3d(mplot3d.art3d.Poly3DCollection(tgtMesh.vectors))
print('pc shape: ', pc.shape)
print('Mean: ', np.mean(pc, axis=0))
# ax.scatter(pc[:,0], pc[:,1], pc[:,2])
# ax.set_xlim([-0.2, 0.2])
# ax.set_ylim([-0.2, 0.2])
# ax.set_zlim([-0.2, 0.2])
# plt.show()



# draw grasps

import mayavi.mlab as mlab
import pickle
from utils.visualization_utils import draw_scene

data = pickle.load(open('temp/mustard.pkl', 'rb'))

generated_grasps = data['grasps']
generated_scores = data['scores']
# pc = data['point_cloud']
pc_colors = np.ones([pc.shape[0], 3])*127

mlab.figure(bgcolor=(1, 1, 1))
draw_scene(
    pc,
    pc_color=pc_colors,
    grasps=[generated_grasps[15]],
    grasp_scores=[generated_scores[15]],
)
mlab.show()

