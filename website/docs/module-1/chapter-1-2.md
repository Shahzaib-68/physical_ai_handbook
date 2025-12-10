---
sidebar_position: 2
slug: /module-1/chapter-1-2
title: Chapter 1.2 - Nodes, Topics, and Services
---

# Chapter 1.2: Nodes, Topics, and Services

## Overview

In the Robot Operating System (ROS), communication between processes occurs through the publish-subscribe paradigm and request-response mechanisms. Understanding these communication patterns is fundamental to building effective robotic systems. This chapter explores the core communication abstractions: nodes, topics, and services.

## Learning Objectives

By the end of this chapter, students will be able to:
- Describe the roles of nodes, publishers, and subscribers in ROS
- Implement basic publisher and subscriber applications
- Explain the difference between topics and services
- Design proper message types for communication

## Nodes: The Foundation of ROS Communication

Nodes are individual processes that perform computation in ROS. All computation in a ROS system happens within nodes, which are organized in a peer-to-peer network.

### Characteristics of Nodes:
- Each node has a unique name within the ROS graph
- Nodes communicate with other nodes by publishing messages to topics or subscribing to topics
- Nodes can also provide or use services
- Nodes can be written in different programming languages and still communicate with each other

### Creating a Node

In most ROS client libraries, creating a node involves:
1. Initializing the ROS client library
2. Instantiating a node object
3. Defining publishers, subscribers, and clients/servers
4. Spinning the node to process callbacks
5. Cleaning up resources when done

Example in Python (`rclpy`):
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish-Subscribe Communication

Topics implement an asynchronous, one-to-many communication pattern. Publishers send messages to topics, and subscribers receive messages from topics.

### Topic Characteristics:
- Asynchronous: Publishers and subscribers don't need to be active simultaneously
- Loose coupling: Publishers don't know who subscribes and vice versa
- Many-to-many: Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic
- Message types: Each topic has a well-defined message type that must be agreed upon

### Message Types

Messages are data structures that are sent across topics. They are defined using `.msg` files in ROS packages. Common primitive types include:
- `bool`
- `int8`, `int16`, `int32`, `int64`
- `uint8`, `uint16`, `uint32`, `uint64`
- `float32`, `float64`
- `string`
- `time`, `duration`

Custom message types can be composed from these primitives and other custom messages.

### Quality of Service (QoS) in Topics

QoS settings allow for fine-tuning the communication behavior:

#### Reliability Policy
- `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`: Messages may be lost
- `RMW_QOS_POLICY_RELIABILITY_RELIABLE`: All messages transmitted reliably

#### Durability Policy
- `RMW_QOS_POLICY_DURABILITY_VOLATILE`: Only new messages are received
- `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL`: Late joiners receive previous messages

## Services: Request-Response Communication

Services implement synchronous, one-to-one communication where a client sends a request and waits for a response from a server.

### Service Characteristics:
- Synchronous: The caller blocks until it receives a response
- One-to-one: A specific service request goes to a specific service server
- Request-Response: Each call has a structured request and response
- Type Safety: Service types define both request and response structures

### Service Types

Services are defined using `.srv` files, which have two parts separated by "---":
- Request: Parameters sent from client to server
- Response: Values sent from server back to client

Example service definition (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

## Practical Exercise: Publisher-Subscriber Example

Let's implement a simple publisher and subscriber:

### Publisher Code (Python):
```python
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(String, 'chatter', 10)
    
    msg = String()
    
    i = 0
    while rclpy.ok():
        msg.data = f'Hello World: {i}'
        node.get_logger().info(f'Publishing: "{msg.data}"')
        publisher.publish(msg)
        i += 1
        
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Code (Python):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Communication: Actions

Beyond topics and services, ROS also supports actions for long-running tasks with feedback:

- **Goals**: Requests for long-running tasks
- **Results**: Final outcomes of completed tasks  
- **Feedback**: Updates during task execution

Actions are appropriate for operations like navigation, grasping, or other complex behaviors that take time to complete.

## Design Considerations

### Topic Design
- Choose appropriate message frequency to avoid network congestion
- Use appropriate QoS settings for reliability and durability
- Design message schemas to minimize bandwidth usage
- Consider message backward compatibility when evolving interfaces

### Service Design
- Only use services for operations that can complete in a reasonable time
- Avoid services for continuous data streams (use topics instead)
- Design services with clear, atomic responsibilities

## Summary

Nodes, topics, and services form the fundamental communication mechanisms in ROS. Topics enable asynchronous, loosely-coupled communication between multiple nodes, while services provide synchronized, request-response interactions. Understanding these communication patterns is essential for designing robust robotic systems that efficiently exchange information.

## Exercises

1. Implement a temperature monitoring system with a sensor node publishing to a temperature topic and a display node subscribing to it.
2. Create a service that calculates the distance between two points in 2D space.
3. Design message types for a robot navigation system that reports battery levels, current position, and obstacle detection.