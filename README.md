# RocketSim

Browser rocket intercept sandbox built with plain Three.js and a single HTML file.

The sim launches a slender rocket from a pad, lets you fly it manually with thrust vector control, and can switch into an auto-target mode that chases moving aircraft using a real guidance loop. The HUD includes a live guidance panel that shows values coming from the active controller, not placeholder telemetry.

## Preview

![RocketSim stand view](docs/stand.png)

### Plane targeting
![Plane targeting](docs/close.png)

![](docs/closer.png)

## What It Does

- Manual flight with throttle and thrust-vector control
- Auto-target intercept mode against moving aircraft
- Aerodynamic canards, fin stabilization, and terrain-aware guidance
- Follow/free camera plus a live guidance HUD

## Controls

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

## Control Loop

The real guidance logic is in [`index.html`](index.html), mainly:

- `updateTargetPrediction()`
- `updateTargetingController()`
- `computeActuatorCommandsForDesiredAxis()`
- `stepPhysics()`

The sim runs at a fixed `240 Hz` step. Each step:

1. updates target aircraft
2. predicts the intercept point
3. builds a guidance acceleration command
4. turns that into gimbal and canard commands
5. applies forces and torques in physics

In short:

- `updateTargetPrediction()` filters target velocity and estimates a lead point
- `updateTargetingController()` mixes PN-style steering, direct terminal pursuit, speed control, and terrain clearance
- `computeActuatorCommandsForDesiredAxis()` converts desired attitude into TVC and canard commands
- `stepPhysics()` applies thrust, aero, canard/fin forces, angular response, and ground contact

## Guidance Panel

The guidance panel shows live values from the active loop, including:

- mode, range, `tgo`, and closing speed
- LOS rate and direct-pursuit blend
- lateral command versus limit
- speed target and throttle command
- TVC/canard commands and terrain margin

If it is hidden, press `I`.
