# BlueROV2 in Gazebo Garden

This is a model of the BlueROV2, including support for both the base and heavy
configurations, that runs in Gazebo Garden. It uses the BuoyancyPlugin,
HydrodynamicsPlugin and ThrusterPlugin.

![BlueROV2 Gazebo](images/bluerov2.png)

## Requirements

- OpenCV:

  - Debian `sudo apt install libopencv-dev`
  - macOS `brew install opencv`

- [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
- [ArduSub and MAVProxy](https://ardupilot.org/dev/docs/building-setup-linux.html)

## Building

A helper script is provided to build the models and plugins:

```bash
./build.sh
```

## Running the simulation

An example can be launched using the following commands:

```bash
./launch.sh
```

Once Gazebo has been launched, you can directly send thrust commands to the BlueROV2
model in Gazebo:

```bash
scripts/cw.sh <model_name>
scripts/stop.sh <model_name>
```

Now launch ArduSub and ardupilot_gazebo:

```bash
cd PATH_TO_ARDUSUB
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f <frame> --model=JSON --out=udp:0.0.0.0:14550 --console
```

where `<frame>` is replaced with either `vectored` for the BlueROV2 base configuration or
`vectored_6dof` for the BlueROV2 Heavy configuration.

Note: if you run into problems switching between the vectored and vectored_6dof frame add the `-w` option to delete all ArduSub parameters.

Use MAVProxy to send commands to ArduSub:

```bash
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
rc 5 1550
disarm
```

Additional information regarding the usage of each model may be found in a model's
respective directory.

> See original README.md at https://github.com/clydemcqueen/bluerov2_ignition
