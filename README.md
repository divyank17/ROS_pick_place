# ROS_pick_place
# Pick and Place Robot with MoveIt and ROS 2

## Overview

This project showcases a robotic pick-and-place operation using MoveIt and ROS 2. It demonstrates how to control a robot arm to pick up an object from one location and place it at another using MoveIt’s planning capabilities.

## Features

- Robot arm manipulation using MoveIt.
- ROS 2 integration for robotic control and node management.
- Ability to move objects between specified locations.

## Prerequisites

Before you start, ensure you have the following installed:

- **ROS 2 Humble**: [Installation Instructions](https://docs.ros.org/en/humble/Installation.html)
- **MoveIt**: [Installation Guide](https://moveit.ros.org/install/)
- **C++17**: Required for building the project.

## Setup Instructions

### 1. Install ROS 2 Humble

Follow the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html) to install ROS 2.

### 2. Install MoveIt

Install MoveIt by following the [MoveIt Installation Instructions](https://moveit.ros.org/install/).

### 3. Create and Set Up the ROS 2 Workspace

Open a terminal and run the following commands:

```bash
mkdir -p ~/pick_and_place_ws/src
cd ~/pick_and_place_ws/src
git clone https://github.com/yourusername/pick_and_place.git
cd ~/pick_and_place_ws
