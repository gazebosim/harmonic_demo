# harmonic_demo

Usage:

```
git clone https://github.com/gazebosim/harmonic_demo

cd harmonic_demo

export GZ_SIM_RESOURCE_PATH=`pwd`/harmonic_demo:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/harmonic_demo/CartPole/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH 

gz sim -v 4 "harmonic_demo/harmonic.sdf"
```

For the Cart-Pole demo, the python modules `numpy` and `scipy` are required. On ubuntu, they can be installed with

```
sudo apt install python3-scipy python3-numpy
```
