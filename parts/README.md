# Parts Folder – Ballerina Cappucina Robot

This folder contains the CAD files for the **Ballerina Cappucina** robot, detailing the mechanical components required for assembly. All files are in **STEP format** and can be 3D-printed or machined as needed.

## Components

- **`Ball_rotator.step`** – Mechanism responsible for rotating the ball.  
  Conceptually similar to a revolving door (like those in hotels), with a servo motor mounted at the top to drive the rotation.

- **`Base.step`** – The main chassis base that serves as the foundation for mounting all other components, including electronic equipment such as servo motors, controllers, and sensors.

- **`Circular Chassis.step`** – The circular frame that forms the structure of the robot’s chassis.

- **`motorcoupler.step`** – Coupler for connecting motors to the omnidirectional wheels.  
  Our motors feature a D-shaped rotor (6mm diameter), while the wheel hub has a 12mm diameter with a T-shaped protrusion to lock the wheel. The coupler ensures secure connection and prevents slippage during operation.

- **`motorholder.step`** – Holder for the motors, suspended underneath the rectangular base to maintain proper alignment and stability.

## Notes

- All CAD files are provided in **STEP format**.  
- Designed for **3D printing or machining**, depending on available resources.  
- The files are intended to facilitate precise assembly of the mechanical components of the Ballerina Cappucina robot.

---

## Assembly Overview

Simplified schematic of the mechanical layout:

          [Ball_rotator]
                │
                │
         ┌──────┴──────┐
         │   Circular  │
         │   Chassis   │
         └──────┬──────┘
                │
        ┌───────┴───────┐
        │      Base      │
        │  (Electronics: │
        │  Servo,ESP32,  │
        │  Jetson Nano  )│
        └───────┬───────┘
        │               │
   [Motorholder]     [Motorholder]
        │               │
       [Motor]          [Motor]
        │               │
[Omnidirectional Wheel] [Omnidirectional Wheel]

