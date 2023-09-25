# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from scipy import linalg

# Controller adapted from https://towardsdatascience.com/comparing-optimal-control-and-reinforcement-learning-using-the-cart-pole-swing-up-openai-gym-772636bc48f4
class LqrController(object):

    def __init__(self, mass_cart, mass_point_mass, pole_length):

        a = 9.81/(pole_length * 4.0/3 - mass_point_mass/(mass_point_mass +
            mass_cart))

        A = np.array([[0, 1, 0, 0],
                      [0, 0, a, 0],
                      [0, 0, 0, 1],
                      [0, 0, a, 0]])

        b = -1/(pole_length*(4.0/3 - mass_point_mass/(mass_point_mass + mass_cart)))
        B = np.array([[0], [1 / (mass_point_mass + mass_cart)], [0], [b]])

        R = np.eye(1)
        Q = np.eye(4)
        Q[0, 0] = 10
        Q[1, 1] = 10

        # solve ricatti equation
        P = linalg.solve_continuous_are(A, B, Q, R)

        # calculate optimal controller gain
        self.K = np.dot(np.linalg.inv(R),
                        np.dot(B.T, P))

    def compute(self, x):
        u = -np.dot(self.K, x)
        return u
