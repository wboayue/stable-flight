# Stable Flight SITL

Stable Flight Software in the Loop (SITL) marks the beginning of my journey in developing a flight controller in Rust. I chose Rust because it offers unique advantages for systems programming. If you are interested in learning more about my decision, you can read a detailed explanation [here](https://wboayue.com/posts/flight-controller-why-rust/).

I decided to use simulation because my previous experience building drones taught me that they require a significant time investment. Even without a custom flight controller, tuning a new build carries a high risk of crashing. I wanted to focus on perfecting the flight control logic without the worry of rebuilding hardware. This approach proved valuable as my simulated drone crashed many times before achieving stable flight.

Currently, my flight controller supports software-in-the-loop simulation with the RealFlight simulator. It is designed to be easily extended to support other flight simulators as well. I chose RealFlight for its realistic physics model, its API for external drone control, its wide selection of RC aircraft, and its ability to define custom models. Although RealFlight is only available on Windows, I have successfully run it on my Mac using Parallels.

Controller logic was initially developed using the following models
* [QuadcopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX)
* [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation)

Ardupilot also maintains a [collection of models](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models).
