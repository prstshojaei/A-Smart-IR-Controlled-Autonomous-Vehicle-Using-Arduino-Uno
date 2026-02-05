# A-Smart-IR-Controlled-Autonomous-Vehicle-Using-Arduino-Uno

This project was developed as part of a coursework project at the **University of Northampton (UK)**.  
It implements a small-scale vehicle that operates in **two different modes**: manual control and autonomous navigation.

---

## Project Overview
The vehicle is designed to work in:
- **Manual Mode:** controlled using an **IR remote control**
- **Autonomous Mode:** obstacle avoidance using an **ultrasonic distance sensor**

A push button allows switching between modes, and a **16×2 LCD** displays the current mode, movement direction, and distance information in real time.

---

## Key Features
- Dual operating modes: **Manual + Autonomous**
- IR remote-based motion control (forward, backward, left, right, stop)
- Autonomous obstacle detection and avoidance routine
- Safety lock to prevent collisions in manual mode
- Real-time status display using LCD
- Simple, low-cost, and educational design

---

## Hardware Components
- **Arduino Uno**
- **Ultrasonic distance sensor**
- **IR receiver and IR remote**
- **L293D motor driver**
- **2× DC motors (differential drive)**
- **16×2 LCD**
- **Push button** (mode selection)
- External power supply (e.g., 9V battery)

---

## System Operation
### Manual Mode
- The vehicle responds to commands sent via IR remote.
- The ultrasonic sensor remains active as a safety mechanism.
- If an obstacle is too close, the vehicle stops automatically.

### Autonomous Mode
- The vehicle moves forward while continuously measuring distance.
- When an obstacle is detected within a predefined threshold, it performs an avoidance routine:
  **stop → turn right → continue forward**
