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
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.vector2d_pb2 import Vector2d
from gz.msgs10.empty_pb2 import Empty
import numpy as np
from threading import Lock
import importlib
from . import lqr_controller


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
        self.init_controller()

        self.node = Node()
        reset_angle_topic = sdf.get_string("reset_angle_topic", "reset_angle")[0]
        print("Subscribing to", reset_angle_topic)
        self.node.subscribe(Double, reset_angle_topic, self.reset_angle_cb)

        self.new_reset_angle = None
        self.reset_angle_lock = Lock()

        reload_controller_topic = sdf.get_string("reload_controller_topic", "reload_controller")[0]
        print("Subscribing to", reload_controller_topic)
        self.node.subscribe(Empty, reload_controller_topic, self.reload_controller_cb)
        self.controller_lock = Lock()

        state_topic = sdf.get_string("state_topic", "state")[0]
        position_topic = state_topic + "/position"
        velocity_topic = state_topic + "/velocity"
        print(f"Advertising to {position_topic} and {velocity_topic}")
        self.position_pub = self.node.advertise(position_topic, Vector2d)
        self.velocity_pub = self.node.advertise(velocity_topic, Vector2d)

    def init_controller(self):
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

        with self.reset_angle_lock:
            if self.new_reset_angle is not None:
                self.pole_joint.reset_position(ecm, [self.new_reset_angle])
                self.new_reset_angle = None

        x = np.array([
            self.cart_joint.position(ecm)[0],
            self.cart_joint.velocity(ecm)[0],
            self.pole_joint.position(ecm)[0],
            self.pole_joint.velocity(ecm)[0]
        ])

        self.publish_state(x)

        with self.controller_lock:
            u = self.controller.compute(x)

        self.cart_joint.set_force(ecm, [u])

    def reset_angle_cb(self, msg):
        with self.reset_angle_lock:
            self.new_reset_angle = msg.data
            print("Resetting angle to", self.new_reset_angle)

    def reload_controller_cb(self, msg):
        with self.controller_lock:
            print("Reloading controller")
            importlib.reload(lqr_controller)
            self.init_controller()

    def publish_state(self, x):
        position_msg = Vector2d()
        position_msg.x = x[0]
        position_msg.y = x[2]

        velocity_msg = Vector2d()
        velocity_msg.x = x[1]
        velocity_msg.y = x[3]

        self.position_pub.publish(position_msg)
        self.velocity_pub.publish(velocity_msg)


def get_system():
    return CartPoleSystem()

