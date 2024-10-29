"""
# This script is to simply illustrate the view of a Lidar
# Please read the guideline before you run this script.
"""

import numpy as np
import utils as utils
import os
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

center = [0, 0, 0]  # Center of the sphere
# sparse
# TODO: modify this function, adding parameters including position and radius
pnts = utils.getPoints(utils.makeSphere(center, 20))

pnts = utils.makePointCloud(pnts)
utils.show_in_vtk([pnts])


# dense
pnts = utils.getPoints(utils.makeSphere(
    center, 20, res_theta=100, res_phi=100))
pnts = utils.makePointCloud(pnts, radius=0.1)
utils.show_in_vtk([pnts])

# cut off
pnts = []
ceil = 0.3
for p in utils.getPoints(utils.makeSphere(center, 1, res_theta=100, res_phi=100)):
    if abs(p[2]) < ceil:
        pnts.append(p)
pnts = np.array(pnts) * 20
pnts = utils.makePointCloud(pnts, radius=0.1)
utils.show_in_vtk([pnts])
