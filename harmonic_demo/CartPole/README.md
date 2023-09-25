# Cart-Pole Demo

The Cart-Pole demo showcases how Gazebo systems can be written in Python. It implements an LQR controller, to balance 
the Cart-Pole such that the pole is always upright.

The plugin subscribes to the `cart_pole/reset_angle` topic which can be used to repeatedly reset the pole angle and let
the controller balance the cart-pole.

```
gz topic -t "cart_pole/reset_angle" -m gz.msgs.Double -p "data: 0.3"
```

*Note*: Large angles will cause the controller to fail, in which case, the simulation has to be reset.

This demo also shows that by putting the LQR controller implementation in a separate module, the plugin can provide a
mechanism for hot reloading the controller while the simulator is running. This can be used, for example, to tune
parameters, or quickly iterate on controller designs. To try this, modify the controller gains `R` and `Q` in `lqr_controller.py`
and reload the controller by publishing to the `cart_pole/reload_controller` topic

```
gz topic -t "cart_pole/reload_controller" -m gz.msgs.Empty -p ""
```

The state of the plant can also be monitored by subscribing to `cart_pole/state/position` and `cart_pole/state/velocity`.
These topics each publish a `Vector2d` where `x` corresponds to the state of the cart and `y` corresponds to the state 
of the pole.

