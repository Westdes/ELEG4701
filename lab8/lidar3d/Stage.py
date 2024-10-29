"""
# Please read the guideline before you run this script.
"""

import numpy as np
import utils as utils
import vtkmodules.all as vtk
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)


class Stage3d:
    def __init__(self):
        self.mesh = vtk.vtkPolyData()
        self.appendPolyFilter = vtk.vtkAppendPolyData()
        self.actor = vtk.vtkAssembly()

    def addObject(self, mesh):
        if isinstance(mesh, str):
            mesh = self.getPolyfromFile(mesh)

        if isinstance(mesh, vtk.vtkPolyData):
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(mesh)
            mesh = utils.makeActor(mesh)
            self.actor.AddPart(mesh)
        mesh = mesh.GetMapper().GetInput()
        self.appendPolyFilter.AddInputData(mesh)
        self.appendPolyFilter.Update()
        self.mesh = self.appendPolyFilter.GetOutput()

    @staticmethod
    def make_default_stage3d():
        stage = Stage3d()
        stage.addObject(utils.makeCube(center=[0, 0, 0], shape=[10, 100, 100]))
        stage.addObject(utils.makeCube(center=[0, 0, 0], shape=[100, 10, 10]))
        stage.addObject(utils.makeCube(center=[0, 0, 0], shape=[500, 500, 1]))
        stage.addObject(utils.makeSphere(center=[30, 30, 30], radius=20))
        # return Stage3d.make_your_stage3d()
        return stage

    @staticmethod
    def make_your_stage3d():
        stage = Stage3d()
        np.random.seed(5)
        cube = utils.makeCube(center=[0, 0, 0], shape=[10, 100, 100])
        ##
        cube_pos = [[-120, 250, 0], [0, 20, 0], [
            -50, 70, 0], [-30, 120, 0], [40, -120, 0]]
        for i in range(5):
            # TODO 1: Fill in needed parameters & modify the code to make your own stage
            # Including both cube and sphere in your stage.
            cube_rot = np.random.randint(-5, 1500, size=3)
            # define rot using np.random.randint, and pos
            cube1 = utils.transModel(cube, rot=cube_rot, pos=cube_pos[i])
            stage.addObject(cube1)
        stage.addObject(utils.makeCube(center=[0, 0, 0], shape=[500, 500, 1]))

        # TODO 2: makeSphare by using addObject, then define radius
        sphere = utils.makeSphere(center=[20, 70, 20], radius=20)
        # TODO 3: define cube2 with makeCube with new rot and pos (use transModel here)
        stage.addObject(sphere)
        return stage


if __name__ == '__main__':
    # s = Stage3d.make_default_stage3d()
    s = Stage3d.make_your_stage3d()
    utils.show_in_vtk([s.actor])
