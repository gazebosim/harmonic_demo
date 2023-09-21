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

from gz.sim8 import Model, Link, Joint
import numpy as np
from . import lqr_controller

# Ideas:
# - Topic to reset initial angle
# - Put controller code in another module and provide a gz-transport service to
# reload it, so we can easily tune gains.
# - Publish state and plot it
# -

class CartPoleSystem(object):

    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity)
        self.link = Link(self.model.link_by_name(ecm, "cart"))
        self.cart_joint = Joint(self.model.joint_by_name(ecm, "cart_joint"))
        self.pole_joint = Joint(self.model.joint_by_name(ecm, "pole_joint"))

        initial_angle = sdf.get_double("initial_angle", 0)[0]

        assert self.cart_joint.valid(ecm)
        assert self.pole_joint.valid(ecm)

        self.cart_joint.enable_position_check(ecm)
        self.pole_joint.enable_position_check(ecm)
        self.cart_joint.enable_velocity_check(ecm)
        self.pole_joint.enable_velocity_check(ecm)

        self.pole_joint.reset_position(ecm, [initial_angle])


        # TODO Get these from the model
        mass_cart = 0.2
        mass_point_mass = 0.03
        pole_length = 0.8
        self.controller = lqr_controller.LqrController(mass_cart,
                mass_point_mass, pole_length)


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

        u = self.controller.compute(x)

        self.cart_joint.set_force(ecm, [u])


def get_system():
    return CartPoleSystem()

