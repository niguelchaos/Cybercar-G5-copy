# Cybercar G5

## Introduction
This repository contains a copy of the source code for navigating a Kiwi car through an intersection, done as part of an academic project. The Kiwi car is a miniature robotic vehicle platform consisting of a Beaglebone board and a Raspberry Pi, alongside a myriad of sensors and motors. More information can be found here: [Chalmers Revere's Kiwicar] (https://github.com/chalmers-revere/opendlv-tutorial-kiwi)

### Project Goals
The general task of the project was to handle an intersection as a self driving vehicle, from beginning to end.
1. Drive up to an intersection, without crashing into other cars. If there is a leading car, maintain safe distance.
3. Stop the car when the car reaches a stop sign.
4. Follow the directional signs - if right of left turns are not allowed, the car will refuse to go in that direction.
5. Determine when to leave - let other cars pass first.
6. Leave the intersection, avoiding obstacles and stopping if needed.

### Microservices

To achieve the project goal, microservices were developed. Microservices are loosely coupled and could run independently, allowing us to continously deliver small pieces as the project progressed.

Each microservice focuses on a particular job:

1. **MoveCar** - Well, moves the car. Specifically handles sensor readings, driving and steering the car. Receives messages from other microservices to know how to move.
2. **AccSafeDistance** - Handles the color and car detection of the leading car, adjusts speed and direction to maintain a safe distance from it.
3. **StopSignRecognition** - Detects Stop signs and notifies MoveCar of them.
4. **CarDetection** - Determines when to leave the intersection. This is done by detecting and tracking the cars on the intersection.
5. **YieldSignDetector** - Retrofitted with detecting directional signs instead of yield due to time constraints.
6. **InputDirection** - Handles the direction to leave when leaving the intersection. Currently requires human interaction to input direction, but will take directional signs into account, refusing if a certain direction is not allowed.

~~We aim to give this car some personality. And collision detection solely for the purpose of deliberately crashing into other cars.~~
