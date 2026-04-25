# RocketSim

This repo now contains two intercept sims:

- A browser version in [index.html](/Users/vidvudscalitis/Desktop/CODING/3dRocket/index.html) built with plain Three.js
- A native C + raylib version in [c](/Users/vidvudscalitis/Desktop/CODING/3dRocket/c)

The browser version is still available at [rocketsim.vidvuds.com](https://rocketsim.vidvuds.com/).

## Browser Sim

The original browser sim launches a rocket from a pad, supports manual thrust-vector control, and can switch into an auto-target mode that chases moving aircraft with a real guidance loop. The HUD exposes live guidance values instead of placeholder telemetry.

### Browser Preview

![RocketSim stand view](docs/stand.png)

![Plane targeting](docs/close.png)

![](docs/closer.png)

### Browser Controls

- `Q / E`: throttle up or down
- `Arrow keys`: manual gimbal
- `T`: toggle auto-target
- `N`: next target
- `Y`: recenter commands
- `V`: cut throttle
- `R`: reset
- `P`: pause
- `LMB / RMB / wheel`: orbit, pan, zoom
- `W A S D`: free-camera pan
- `X / Z`: camera up or down
- `C`: return to follow camera
- `I`: toggle guidance panel
- `Tab`: toggle help

### Browser Guidance

The main browser guidance loop lives in [index.html](/Users/vidvudscalitis/Desktop/CODING/3dRocket/index.html), primarily:

- `updateTargetPrediction()`
- `updateTargetingController()`
- `computeActuatorCommandsForDesiredAxis()`
- `stepPhysics()`

The browser sim runs at a fixed `240 Hz` step. Each step updates target aircraft, predicts intercept, builds a guidance acceleration command, converts it into TVC/canard commands, and applies forces and torques.

## C + raylib Sim

The native sim in [c](/Users/vidvudscalitis/Desktop/CODING/3dRocket/c) is a ground-to-air missile intercept sandbox. It loads the included missile model, uses the low-poly MIG STL target, predicts an intercept point, and flies the missile against a moving aircraft with simplified atmosphere, drag, thrust, and turn-limit modeling.

### C Sim Preview

![Ground launch view](docs/groundC.png)

![Intercept view](docs/interceptC.png)

![Close-up view](docs/closeupC.png)

### Build and Run

```sh
cd /Users/vidvudscalitis/Desktop/CODING/3dRocket/c
make
./rocket_sim
```

### C Sim Controls

- `I`: launch interceptor
- `Space`: pause or resume
- `Left / Right`: halve or double time scale
- `Down`: set time scale to `0.10x`
- `Up`: reset time scale to `1.00x`
- Double-click window: toggle borderless fit-to-screen
- Mouse drag: orbit camera around the missile
- Mouse wheel: zoom
