---
sidebar_position: 1
slug: /module-1/chapter-1-1
title: Chapter 1.1 - Introduction to ROS 2 Architecture
---

# Chapter 1.1: Introduction to ROS 2 Architecture

## Overview

The Robot Operating System 2 (ROS 2) is the successor to the popular ROS framework, designed to address the limitations of its predecessor and provide a more robust, scalable, and secure platform for robotic development. Unlike the original ROS, ROS 2 is built on top of DDS (Data Distribution Service), which offers improved communication capabilities and reliability.

## Learning Objectives

By the end of this chapter, students will be able to:
- Explain the fundamental differences between ROS 1 and ROS 2
- Describe the layered architecture of ROS 2
- Understand the role of DDS in ROS 2 communication
- Identify the core components of a ROS 2 system

## What is ROS 2?

ROS 2 is a middleware framework that provides services designed for a heterogeneous computer cluster, including hardware abstraction, device drivers, implementation of commonly used functionality, message-passing between processes, and package management. It is designed to support multiple programming languages and operating systems, making it ideal for complex robotic applications.

### Key Features of ROS 2
- **Real-Time Support**: Ability to achieve deterministic behavior with real-time guarantees
- **Multi-Platform Support**: Runs on Linux, macOS, Windows, and even embedded systems
- **Enhanced Security**: Built-in security features to protect robotic systems
- **Scalability**: Designed to work in distributed systems with multiple nodes
- **Improved Communication**: Uses DDS for reliable message passing

## Differences Between ROS 1 and ROS 2

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS-based |
| Master | Required master node | No central master |
| Real-time | Limited support | Native real-time support |
| Security | No built-in security | Integrated security |
| Platforms | Primarily Linux/macOS | Linux, macOS, Windows, embedded |

### The Need for ROS 2

ROS 1 had several limitations that became apparent as robotics evolved:
- Single point of failure due to the central master node
- Limited support for real-time systems
- Lack of security features
- Difficulty deploying across multiple platforms
- Scalability challenges in large systems

## Core Architecture of ROS 2

The architecture of ROS 2 consists of several layers:

### 1. Application Layer
This layer contains the user's robotic applications built using ROS 2 client libraries. It includes the nodes, launch files, and other application-specific components.

### 2. Client Library Layer
ROS 2 supports multiple client libraries:
- **rclcpp** for C++
- **rclpy** for Python
- **rclrs** for Rust
- **rclc** for microcontrollers
- And others for Java, etc.

These libraries provide language-specific APIs to interact with the ROS 2 middleware.

### 3. ROS Client Library (rcl) Layer
This is a thin C layer that abstracts the underlying middleware implementation. It provides common functionality across all client libraries.

### 4. Middleware Layer
The middleware layer implements the DDS interface and handles all communication between nodes. It manages message distribution, Quality of Service (QoS) policies, and network protocols.

## DDS and Its Role in ROS 2

DDS (Data Distribution Service) is a vendor-neutral, open standard for real-time systems. It provides an interoperable data exchange layer that enables devices to communicate directly with one another, without requiring a central server.

### Advantages of DDS over ROS 1's communication model:
- **Decentralized**: No single point of failure
- **Discovery**: Automatic discovery of available topics, services, and actions
- **Quality of Service**: Configurable policies for reliability, durability, etc.
- **Interoperability**: Multiple vendors implement DDS, ensuring compatibility

### DDS Quality of Service (QoS) Policies
QoS policies define how messages are delivered and what guarantees are provided. Key policies include:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep last or keep all
- **Rate**: Publish rate limits

## Practical Exercise: Setting Up ROS 2

To understand ROS 2 architecture, let's install ROS 2 and explore its basic components.

### Installation Options:
1. Binary packages (recommended for beginners)
2. From source (for developers)
3. Docker images (for isolated environments)

### Key Directories and Files:
- `$ROS_DISTRO`: Defines the active ROS 2 distribution
- `$AMENT_PREFIX_PATH`: Contains the installation prefixes
- `/opt/ros/$ROS_DISTRO`: Default installation directory

```
ros2 run <package_name> <executable_name>
```

This command demonstrates how ROS 2 finds and executes programs based on the package structure.

## Summary

In this chapter, we explored the foundational architecture of ROS 2, understanding how it differs from ROS 1 and why these changes were necessary. The adoption of DDS provides a more robust and scalable communication infrastructure, while the layered architecture allows for language independence and easier maintenance.

Understanding the ROS 2 architecture is crucial for effectively developing robotic applications, as it determines how different components interact and how messages flow between them.

## Further Reading

- Official ROS 2 documentation: www.ros.org
- DDS specification: www.omg.org/spec/DDS
- ROS 2 design documents: design.ros2.org