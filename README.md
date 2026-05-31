<div align="center">

# 📡 Sensor Fusion — LiDAR Obstacle Detection

### 3D Point Cloud Processing for Autonomous Vehicles

![C++](https://img.shields.io/badge/C++-Autonomous_Systems-blue?style=for-the-badge&logo=cplusplus)
![LiDAR](https://img.shields.io/badge/LiDAR-3D_Perception-green?style=for-the-badge)
![Sensor Fusion](https://img.shields.io/badge/Sensor_Fusion-Obstacle_Detection-orange?style=for-the-badge)
![Computer Vision](https://img.shields.io/badge/Computer_Vision-Point_Clouds-red?style=for-the-badge)
![Robotics](https://img.shields.io/badge/Robotics-Perception-purple?style=for-the-badge)

Udacity Sensor Fusion Nanodegree Project

3D obstacle detection using LiDAR point clouds, RANSAC segmentation,
Euclidean clustering, and KD-Tree acceleration.

</div>

---

## Demo

<p align="center">
  <img src="lidar-obstacle-detection.gif" width="850">
</p>

### What you're seeing

- Green points → road surface
- Colored clusters → detected obstacles
- Red bounding boxes → object detections
- Vertical pole on the right correctly isolated as an independent object

The pipeline combines:

- 3D RANSAC segmentation
- Euclidean clustering
- KD-Tree nearest-neighbor search
- Bounding box generation

---

# Overview

Before an autonomous vehicle can make decisions, it must first understand its surroundings.

This project implements a LiDAR-based obstacle detection pipeline capable of identifying vehicles and other obstacles from raw 3D point cloud data.

The system processes LiDAR measurements and transforms them into structured information about the environment through:

- Ground plane segmentation
- Point cloud clustering
- Object detection
- Bounding box generation

The resulting perception pipeline allows the autonomous vehicle to distinguish the road surface from surrounding obstacles and estimate the position of nearby objects.

---

# Project Objectives

The goals of this project were to:

- Process raw LiDAR point cloud data
- Segment the road surface
- Detect obstacles
- Cluster object points
- Generate 3D bounding boxes
- Track object detection consistency across frames

---

# Perception Pipeline

```text
LiDAR Point Cloud
          ↓
Filtering
          ↓
Ground Segmentation
          ↓
Obstacle Extraction
          ↓
Euclidean Clustering
          ↓
Bounding Box Generation
          ↓
Detected Objects
```

---

# Point Cloud Processing

The project operates directly on raw 3D LiDAR measurements.

Each point represents a reflected laser return and contains spatial information describing the environment.

Point clouds provide:

- Object geometry
- Relative position
- Scene structure

without relying on image-based perception.

---

# Ground Segmentation

One of the first challenges in LiDAR perception is separating the road surface from obstacles.

This project uses a custom implementation of:

## 3D RANSAC Plane Segmentation

RANSAC is used to identify the dominant ground plane.

Benefits:

- Robust to noise
- Robust to outliers
- Efficient for large point clouds

The resulting segmentation separates:

✅ Road surface

✅ Obstacles

The implementation follows the 3D RANSAC algorithm developed during the course. :contentReference[oaicite:0]{index=0}

---

# Obstacle Clustering

Once the road plane has been removed, the remaining points correspond to potential obstacles.

The project uses:

## Euclidean Clustering

Nearby points are grouped together into clusters representing individual objects.

The clustering implementation uses:

- Euclidean distance metrics
- KD-Tree acceleration
- Recursive cluster expansion

This approach allows individual vehicles and roadside objects to be isolated from the environment. :contentReference[oaicite:1]{index=1}

---

# KD-Tree Acceleration

To improve performance, the clustering process relies on a custom KD-Tree implementation.

Benefits include:

- Fast nearest-neighbor search
- Reduced computational cost
- Scalable point cloud processing

KD-Trees are widely used throughout robotics, computer vision, and spatial search applications.

---

# Bounding Box Generation

After clustering, the system computes 3D bounding boxes around detected objects.

The final output:

- Encloses vehicles
- Encloses roadside obstacles
- Produces one bounding box per detected object
- Maintains detection consistency across multiple frames

Bounding boxes provide a simplified representation that can later be used for:

- Tracking
- Prediction
- Path planning
- Collision avoidance

---

# Technical Skills Demonstrated

## Sensor Fusion

- LiDAR Processing
- 3D Perception
- Environmental Understanding

## Robotics

- Obstacle Detection
- Spatial Reasoning
- Point Cloud Processing

## Algorithms

- RANSAC
- Euclidean Clustering
- KD-Tree Search

## Software Engineering

- Modern C++
- Numerical Computing
- Real-Time Processing

---

# Repository Structure

```text
src/
├── processPointClouds.cpp
├── processPointClouds.h
├── kdtree.h
├── cluster.cpp
├── segmentation.cpp

data/
├── point_clouds/

README.md
```

---

# Results

The completed pipeline successfully:

✅ Segments the road surface

✅ Identifies obstacle points

✅ Clusters individual objects

✅ Generates bounding boxes

✅ Maintains detection consistency across frames

The resulting perception system provides a simplified but realistic example of LiDAR-based object detection for autonomous vehicles.

---

# Key Concepts Explored

- LiDAR Perception
- Point Clouds
- 3D Geometry
- RANSAC
- Plane Segmentation
- Euclidean Clustering
- KD-Trees
- Obstacle Detection
- Autonomous Driving

---

# Why This Project Matters

Modern autonomous vehicles depend heavily on 3D perception systems.

LiDAR remains one of the most important sensing technologies for:

- Autonomous vehicles
- Mobile robots
- Drones
- Warehouse automation
- Mapping systems

This project demonstrates how raw sensor measurements can be transformed into meaningful environmental understanding.

---

# Related Sensor Fusion Projects

This repository is part of a broader autonomous systems portfolio including:

- Extended Kalman Filter Sensor Fusion
- LiDAR Obstacle Detection
- Kidnapped Vehicle Localization
- Highway Path Planning
- PID Control

Together these projects cover perception, localization, planning, and control for autonomous systems.

---

# Learning Outcomes

This project provided practical experience with:

- LiDAR data processing
- Point cloud segmentation
- Spatial clustering
- Real-time obstacle detection
- Robotics perception pipelines

It also serves as a foundation for more advanced perception systems combining LiDAR, radar, cameras, and deep learning.

---

# Disclaimer

This repository is provided for educational and portfolio purposes.

Students may study the code and reports for learning purposes, but submitting this work as coursework would constitute plagiarism and may violate academic integrity policies.

Copyright © Sabrina Palis
