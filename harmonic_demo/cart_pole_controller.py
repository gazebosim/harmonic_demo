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

from gz.math7 import Vector3d
from gz.sim8 import Model, Link, Joint
import random
import numpy as np
from scipy import linalg


# Controller adapted from https://towardsdatascience.com/comparing-optimal-control-and-reinforcement-learning-using-the-cart-pole-swing-up-openai-gym-772636bc48f4
class TestSystem(object):

    def __init__(self):
        self.id = random.randint(1, 100)

    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity)
        self.link = Link(self.model.link_by_name(ecm, "cart"))
        self.cart_joint = Joint(self.model.joint_by_name(ecm, "cart_joint"))
        self.pole_joint = Joint(self.model.joint_by_name(ecm, "pole_joint"))

        assert self.cart_joint.valid(ecm)
        assert self.pole_joint.valid(ecm)

        self.force = sdf.get_double("force")

        self.cart_joint.enable_position_check(ecm)
        self.pole_joint.enable_position_check(ecm)
        self.cart_joint.enable_velocity_check(ecm)
        self.pole_joint.enable_velocity_check(ecm)


        # TODO Get these from the model
        mass_cart = 0.2
        mass_pole = 0.05
        pole_length = 0.8

        a = 9.81/(pole_length * 4.0/3 - mass_pole/(mass_pole +
            mass_cart))

        A = np.array([[0, 1, 0, 0],
                      [0, 0, a, 0],
                      [0, 0, 0, 1],
                      [0, 0, a, 0]])

        b = -1/(pole_length*(4.0/3 - mass_pole/(mass_pole+ mass_cart)))
        B = np.array([[0], [1 / (mass_pole+ mass_cart)], [0], [b]])

        R = np.eye(1)
        Q = 5*np.eye(4)

        # solve ricatti equation
        P = linalg.solve_continuous_are(A, B, Q, R)

        # calculate optimal controller gain
        self.K = np.dot(np.linalg.inv(R),
                        np.dot(B.T, P))

    def pre_update(self, info, ecm):
        if info.paused:
            return

        if len(self.cart_joint.position(ecm)) == 0:
            return

        x = np.array([
            self.cart_joint.position(ecm)[0],
            self.cart_joint.velocity(ecm)[0],
            self.pole_joint.position(ecm)[0],
            self.pole_joint.velocity(ecm)[0]
            ])

        u = -np.dot(self.K, x)
        u_clipped = np.clip(u, -10000, 10000)

        # print(f"x={x} u={u_clipped}, ")
        self.cart_joint.set_force(ecm, [u_clipped])

        # if info.iterations % 3000 == 0:
        #     self.cart_joint.set_force(ecm, [self.force * (random.random() - 0.5)])


def get_system():
    return TestSystem()

