# ESP32-fluid-simulation

![](images/headliner.jpg)

This is a fluid simulation running on an ESP32, namely the one embedded into a development board that has been dubbed the "Cheap Yellow Display" (CYD) on Brian Lough's Discord channel (monorepo [here](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display)). It simulates a bed of dyed fluid at 28-30 FPS. To do this, Jos Stam's famous technique, featured in "Real-Time Fluid Dynamics for Games", is applied on a *very* small domain (of 80x60, then upscaled to 320x240) with heavy modifications in order to run at high speed. Those modifications include a Poisson solver based on red-black successive over-relaxation (SOR).

As an aside, it isn't what I originally wanted to make: free-surface simulation under gravity. For that, see the project [pi-sph-fluid](https://github.com/colonelwatch/pi-sph-fluid) that I completed much later.

## Running

⚠️ **Disclaimer!** ⚠️ This project may fail to compile or crash after compiling with some versions of the ESP32 Arduino Core. The latest validated version is v3.3.1.

This project is built in Arduino, and the only dependencies are [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) (this requires a custom `User_Setup.h` first, see the monorepo for one working example) and [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen). Vendors for the CYD can be found through the monorepo.

## Really helpful sources

If you're interested in the CYD, the monorepo was critical for me to get started with it.

If you're interested in this specific kind of fluid simulation and you have an understanding of multi-variable calculus, here are some sources I would recommend:

* A beginner-friendly intro: http://jamie-wong.com/2016/08/05/webgl-fluid-simulation/
* An older, more comprehensive guide: https://developer.nvidia.com/gpugems/gpugems/part-vi-beyond-triangles/chapter-38-fast-fluid-dynamics-simulation-gpu
* A helpful guide written by Jos Stam (just Google this): Stam, Jos 2003 *Real-Time Fluid Dynamics for Games*

I also have a series of articles on my personal website, though they may not be entirely up to date. If read from beginning to end, these articles should only require linear algebra and ordinary calculus. They're linked below:

1. [https://kenny-peng.com/2023/07/21/esp32_fluid_sim_1.html](Rebuilding ESP32-fluid-simulation: overview of tasks and intertask communication in FreeRTOS (Part 1))
2. [https://kenny-peng.com/2023/07/30/esp32_fluid_sim_2.html](Rebuilding ESP32-fluid-simulation: the touch and render tasks (Part 2))
3. [https://kenny-peng.com/2023/09/22/esp32_fluid_sim_3.html](Rebuilding ESP32-fluid-simulation: an outline of the sim task (Part 3))
4. [https://kenny-peng.com/2024/01/20/esp32_fluid_sim_4.html](Rebuilding ESP32-fluid-simulation: the advection and force steps of the sim task (Part 4))
5. [https://kenny-peng.com/2024/09/26/esp32_fluid_sim_5.html](Rebuilding ESP32-fluid-simulation: the pressure projection step of the sim task (Part 5))
