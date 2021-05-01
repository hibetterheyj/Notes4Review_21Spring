[toc]

# MICRO-502_Aerial_Robotics_Notes

> Lecture notes by Yujie He
>
> Last updated on 2021/4/29

# Flapping-Wing (week9)

# Drone Regulations (week9)

> Author: Markus Farner
>
> https://www.bazl.admin.ch/bazl/en/home/good-to-know/drohnen.html

- Unmanned Aircraft Systems (UAS) >= Drones; UAS  = Remotely piolted aircraft systems / autonomous aircraft systems

<img src="./pics/aerial/week9_regulation_uas.png" alt="week9_regulation_uas" style="zoom:50%;" />

- Rules in Aviation: Federal Office of Civil Aviation Switzerland

- Everything which is not forbidden is allowed -> Switzerland

  Trust, less difficult for innovation

- 3 Pillar Concept / Drone Categories

  1. Open-Within the legal framework (No Authorization required)
  2. Specific-Not sufficiently safe (Authorization required)
  3. Certified-Approved to accepted standards

- Act

  - **Ordinance on Special Category Aircraft**
    - No authorization required for commercial flights
    - No distinction between Unmanned Aircraft and Model Aircraft
  - **DETEC Ordinance on Special Category Aircraft**
    - No authorization below **30kg**
    - Within direct visual contact (VLOS)
    - Not within a distance <=100m around crowds
  - **ANSP (Skyguide) or Airport responsibility**
    - \> **5km** Distance to civil & military airports/aerodromes
    - < **150m** AGL (Above Ground Level) within a CTR

- Act in EU
  - Open/Specific/Certified
  - Difference
    - restrictions: MTOM **25kg**
    - maximum flying altitude: **120m** 

- Specific Category

  > Application for an operating permit on the basis of the **SORA (Specific Operations Risk Assessment)**

  **Operational Volume = Flight Geography + Contingency Volume **

  <img src="./pics\/aerial/week9_regulation_SORA.jpg" alt="week9_regulation_SORA" style="zoom:60%;" />

- :question: Robustness Levels: Integrity + Assurance

- U-Space

  > The U-space is a collection of decentralized services that collectively aim to safely and efficiently integrate drones into the airspace and enable drone operations alongside manned flight.
  >
  > https://www.bazl.admin.ch/bazl/en/home/good-to-know/drohnen/wichtigsten-regeln/uspace.html.html
  >
  > https://www.skyguide.ch/en/events-media-board/u-space-live-demonstration/

  airspace in block to avoid collision and report the location for further path calculation

# UAS Hardware (week10)

## Introduction

> main component required

1. The aerial vehicle
   - Air frame
   - Actuators for propulsion and control
   - Energy source
   - Autopilot
     - Sensors for attitude estimation
     - Electronics for regulation, control and communication
     - Sensor and avoid system
2. Payload
   - Cameras
   - Environmental sensors (wind, temperature, humidity)
   - Robotic arms for manipulation
3. Ground Control Station
   - Communication systems
   - Interface to monitor internal parameters and to send commands to the vehicle

## Frame and materials

### materials comparison

| Material | Composite | ABS/PLA | Wood | Foam |
| -------- | --------- | ------- | ---- | ---- |
| Pros     |           |         |      |      |
| Cons     |           |         |      |      |
| Comment  |           |         |      |      |

### metric when considering materials

- Young's modulus [[wiki](https://en.wikipedia.org/wiki/Young%27s_modulus)]

  弹性模量，正向应力与正向应变的比值

- Specific modulus [[wiki](https://en.wikipedia.org/wiki/Specific_modulus)]

  比模量，单位密度的弹性模量，劲度－质量比，在航天工业中有广泛应用。

## Energy sources

## Actuators for propulsion and maneuvering

## Propellers

## Sensors

## Autopilots

## Communication protocols