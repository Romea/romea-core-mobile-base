## ROMEA Core Mobile base library

This library offers a comprehensive set of tools for computing both forward and inverse kinematics for four-wheeled vehicles or two-tracked systems. It supports various kinematic models, including one-axle steering, two-axle steering, skid steering, and omni-directional steering. The library enables precise control of actuators and calculates odometry. It also extends these models for simulations. Supported vehicle types include:
- **1FAS2F2WD**: One front-axle steering, two front-wheel drive
- **1FAS2R2WD**: One front-axle steering, two rear-wheel drive
- **1FAS4WD**: One front-axle steering, four-wheel drive
- **2AS2F2WD**: Two-axle steering, two front-wheel drive
- **2AS2R2WD**: Two-axle steering, two rear-wheel drive
- **2AS4WD**: Two-axle steering, four-wheel drive
- **2FWS2F2WD**: Two front-wheel steering, two front-wheel drive
- **2FWS2R2WD**: Two front-wheel steering, two rear-wheel drive
- **2FWS4WD**: Two front-wheel steering, four-wheel drive
- **2TD**: Two track drive
- **4WD**: Four-wheel drive
- **4WS4WD**: Four-wheel steering, four-wheel drive

This library allows flexible vehicle control and extends easily to simulation environments.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-mobile-base/refs/heads/main/romea_mobile_base_public.repos
5. vcs import src < romea_mobile_base_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Mobile Base library was developed by **Jean Laneurit** in the context of various research projects carried out at INRAE.

## **Contact**

If you have any questions or comments about Romea Core Mobile Base library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 
