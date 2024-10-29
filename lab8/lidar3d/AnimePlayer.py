"""
# this script shows how lidar works by animation
# and you can make an reconstruction for you point clouds
# try to make your output as similar as the stage!
"""

import numpy as np
import utils as utils
from LidarCore import Lidar
from Stage import Stage3d
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)


class AnimePlayer:
    def __init__(self, turns, time_speed=5, model=None):
        s = Stage3d.make_your_stage3d()
        self.li = Lidar(s.mesh, model=model)

        self.iterations = turns
        self.renderer, self.win, self.iren = utils.makeVtkRenderWindow()
        actors = [self.li.actor,
                  utils.makeActor(self.li.target_model, opacity=1),
                  self.li.sampled_filter.actor
                  ]
        # to save the points you sampled
        # it will be saved after you closed the window

        for a in actors:
            self.renderer.AddActor(a)
        self.renderer.ResetCamera(actors[1].GetBounds())
        # print(self.li.sampled_pnts.shape)

        # print('the point clouds saved!')
        self.iren.Initialize()
        self.iren.AddObserver('TimerEvent', self.execute)
        self.timerId = self.iren.CreateRepeatingTimer(time_speed)
        self.win.SetSize(800, 800)
        self.iren.Start()

    def execute(self, iren, event):
        if self.iterations == 0:
            iren.DestroyTimer(self.timerId)

        self.li.scan()
        mov = -10  # self.iterations * 10

        self.li.moveInXYPlane(mov, mov)
        # self.li.setLidarPosition([150 + mov, 150 + mov, 10])
        self.renderer.Render()
        iren.GetRenderWindow().Render()
        iren.Render()
        self.iterations -= 1
        # to save the points you sampled
        np.save(os.path.join(os.getcwd(), 'test_pnt.npy'), self.li.sampled_pnts)


def test_anime():
    AnimePlayer(turns=20)


def test_anime_your_model():
    # TODO: set the path and filename of the model; fill the transModel function.
    # Adding a model as the lidar. The model file name should end with .obj
    # If the model is small or need to be rotated use function utils.transModel.
    model = '/home/edward/lab8_project/lidar3d/model/UFO_Empty.obj'  # edit your path
    if os.path.exists(model):
        model = utils.getPolyfromFile(model)

        rot = np.random.randint(0, 360, size=3)
        pos = np.random.uniform(-5, 5, size=3)

        # Apply transformations to the model
        model = utils.transModel(model, rot=rot, pos=pos)
    else:
        model = None
        print('no path or model')
    AnimePlayer(turns=20, model=model)


def showWhatYouSampled():
    # take a screen shot of your sampled feature
    # remember take a photo for the result
    # because the function is time consuming
    print('ready to reconstruction')
    pts = np.load(os.path.join(os.getcwd(), 'test_pnt.npy'))
    print(pts.shape)
    cloud = utils.makeActor(utils.makePointCloud(
        pts, radius=0.3), color=[0, 1, 0])
    pts = utils.numpyArray2vtkPoints(pts)
    # this function is time consuming, please wait
    mesh = utils.reconstructionFromPoints(pts)

    actor = utils.makeActor(mesh, color=[1, 1, 1])
    mapper = actor.GetMapper()
    mapper.ScalarVisibilityOff()
    mapper.Update()

    origin_stage = utils.makeActor(
        Stage3d.make_default_stage3d().mesh, color=[0, 1, 1], opacity=0.1)
    utils.show_in_vtk([actor, cloud, origin_stage])


if __name__ == "__main__":
    # test_anime()  # step1 run this function
    test_anime_your_model()  # step 2 run this function
    # showWhatYouSampled()  # step 3 run this function
