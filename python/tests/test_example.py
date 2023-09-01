import unittest

import numpy as np

import gtsam
import gtsam_gpdepth
from gtsam.utils.test_case import GtsamTestCase


class TestExample(GtsamTestCase):

    def test_GpDepth2D_unwhitenedError(self):
        np.random.seed(0)

        values = gtsam.Values()
        values.insert(1, 1.0)
        values.insert(2, 0.5)
        values.insert(3, 3.0)

        L = np.tril(np.random.rand(3,3))

        noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0]))
        factor = gtsam_gpdepth.GpDepthFactor2D(noise_model, values.keys(), L)

        log_depths = factor.getDataVector(values)
        error = factor.unwhitenedError(values)
        error2 = np.linalg.inv(L) @ log_depths
        self.gtsamAssertEquals(error, error2, tol=1e-9)

        depths = np.exp(log_depths)
        print(depths)
        # print(error)
        # print(error2)

    def test_GpDepth3D_unwhitenedError(self):
        np.random.seed(0)
        
        mean_depth = np.array([0.4])
        mean_log_depth = np.log(mean_depth)
        pose = gtsam.Pose3() # Identity

        values = gtsam.Values()
        values.insert(0, pose)
        values.insert(1, gtsam.Point3(2.0, 1.5, 1.0))
        values.insert(2, gtsam.Point3(4.0, -3.0, 0.5))
        values.insert(3, gtsam.Point3(-1.0, 2.5, 3.0))
        values.insert(4, mean_log_depth)

        pose_key = 0
        landmark_keys = gtsam.KeyVector()
        for i in range(1,4):
            landmark_keys.append(i)
        mean_key = 4

        L = np.tril(np.random.rand(3,3))

        noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0]))
        factor = gtsam_gpdepth.GpDepthFactor3D(noise_model, pose_key, landmark_keys, mean_key, L)

        log_depths_minus_mean = factor.getDataVector(values)
        error = factor.unwhitenedError(values)
        error2 = np.linalg.inv(L) @ log_depths_minus_mean
        
        self.gtsamAssertEquals(error, error2, tol=1e-9)

        depths = np.exp(log_depths_minus_mean + mean_log_depth)
        print(depths)

if __name__ == "__main__":
    unittest.main()
