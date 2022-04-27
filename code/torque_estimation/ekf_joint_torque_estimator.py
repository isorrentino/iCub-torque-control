import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from numpy import array, eye, asarray
import idyntree.bindings as idyn
import bipedal_locomotion_framework.bindings as blf

class EKFJointTorqueEstimator(object):

    def __init__(self, model_path: str, param_handler: blf.parameters_handler.IParametersHandler):

        # Load the reduced model
        model_loader = idyn.ModelLoader()
        considered_joints_idyn = idyn.StringVector()

        for joint in param_handler.get_parameter_vector_string('joint_list'):
            considered_joints_idyn.push_back(joint)

        ok = model_loader.loadReducedModelFromFile(model_path, considered_joints_idyn)
        if not ok:
            msg = "Unable to load the model named from the file: " + model_path + "."
            raise ValueError(msg)

        # store the model and compute the traversals
        self._model = model_loader.model().copy()

        self.rk = ExtendedKalmanFilter(dim_x=7*considered_joints_idyn.size(), dim_z=3*considered_joints_idyn.size())

    def initialize_ekf(self):
        self.rk.F = 0
