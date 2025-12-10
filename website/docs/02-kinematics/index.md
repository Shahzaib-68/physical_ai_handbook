---
sidebar_position: 2
title: "Chapter 2: Kinematics Fundamentals"
description: "Understanding motion in Physical AI and how to simulate it in 3D"
---

import InteractiveDemo from '@site/src/components/InteractiveDemo';
import MakeItYourOwn from '@site/src/components/MakeItYourOwn';

# Chapter 2: Kinematics Fundamentals 🏃‍♂️

In this chapter, you'll learn about kinematics - the study of motion without considering the forces that cause it. Understanding kinematics is crucial for programming robots and physical systems to move properly! Chalo, gati samjhte hain! (Let's understand motion!)

## What is Kinematics? 📐

Kinematics is the branch of physics that describes the motion of objects. It deals with concepts like:
- **Position**: Where something is in space
- **Velocity**: How fast something is moving and in what direction
- **Acceleration**: How quickly the velocity is changing

### Position, Velocity, and Acceleration 📈

In robotics, these concepts are essential for:
- Planning robot trajectories
- Controlling robot movements
- Predicting where a robot will be in the future

## Interactive Kinematics Demo! 🎯

Let's visualize how position, velocity, and acceleration work together in a 3D space:

<InteractiveDemo 
  id="demo2"
  title="3D Motion Visualization"
  description="Visualize position, velocity, and acceleration vectors in 3D"
  code={`// Simple 3D motion simulation using Three.js
// This is a conceptual demo - actual Three.js implementation would be more complex

function setup() {
  createCanvas(400, 400);
  textAlign(CENTER, CENTER);
}

function draw() {
  background(220);
  
  // Draw coordinate system
  stroke(0);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
  
  // Calculate position based on time
  let time = millis() / 1000; // seconds
  let x = width/2 + 100 * cos(time * 0.5);
  let y = height/2 + 50 * sin(time * 0.5);
  
  // Draw position vector
  stroke(255, 0, 0);
  strokeWeight(2);
  line(width/2, height/2, x, y);
  
  // Draw moving object
  fill(0, 0, 255);
  noStroke();
  circle(x, y, 20);
  
  // Draw velocity vector (tangent to motion)
  let velX = -100 * 0.5 * sin(time * 0.5);
  let velY = 50 * 0.5 * cos(time * 0.5);
  stroke(0, 255, 0);
  strokeWeight(2);
  line(x, y, x + velX, y + velY);
  
  // Draw title
  fill(0);
  textSize(16);
  text('Position & Velocity in 2D Motion', width/2, 30);
  
  // Display time
  textSize(12);
  text('Time: ' + time.toFixed(1) + 's', width/2, height - 20);
}`}
  dependencies={['p5.js']}
  challengePrompt="Modify the motion path to create a spiral instead of an ellipse!"
/>

## How Kinematics Applies to Robots 🤖

Robots use kinematics for:

1. **Forward Kinematics**: Knowing the joint angles, determine where the end effector (hand) is
2. **Inverse Kinematics**: Knowing where you want the end effector to be, determine the joint angles needed

### Real-World Example 🌍

When a robot arm needs to pick up an object:
- Sensors determine the object's position (perception)
- Inverse kinematics calculates the required joint angles (planning)
- Motors move the joints to those angles (action)

## Chapter Summary 🧠

In this chapter, you've learned:
1. Kinematics describes motion without considering forces
2. Key concepts: position, velocity, acceleration
3. How kinematics is essential for robot motion planning
4. How to visualize motion in 2D and 3D

## Your Challenge 🏆

<MakeItYourOwn
  userId="demo-user"
  baseDemoId="demo2"
  initialCode={`// Simple 3D motion simulation using Three.js
// This is a conceptual demo - actual Three.js implementation would be more complex

function setup() {
  createCanvas(400, 400);
  textAlign(CENTER, CENTER);
}

function draw() {
  background(220);
  
  // Draw coordinate system
  stroke(0);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
  
  // Calculate position based on time
  let time = millis() / 1000; // seconds
  let x = width/2 + 100 * cos(time * 0.5);
  let y = height/2 + 50 * sin(time * 0.5);
  
  // Draw position vector
  stroke(255, 0, 0);
  strokeWeight(2);
  line(width/2, height/2, x, y);
  
  // Draw moving object
  fill(0, 0, 255);
  noStroke();
  circle(x, y, 20);
  
  // Draw velocity vector (tangent to motion)
  let velX = -100 * 0.5 * sin(time * 0.5);
  let velY = 50 * 0.5 * cos(time * 0.5);
  stroke(0, 255, 0);
  strokeWeight(2);
  line(x, y, x + velX, y + velY);
  
  // Draw title
  fill(0);
  textSize(16);
  text('Position & Velocity in 2D Motion', width/2, 30);
  
  // Display time
  textSize(12);
  text('Time: ' + time.toFixed(1) + 's', width/2, height - 20);
}`}
  challengePrompt="Modify the motion path to create a spiral instead of an ellipse!"
/>

## Next Steps 🚀

In the next chapter, we'll explore dynamics - how forces affect motion. We'll look at how robots interact with objects and how physics simulations can predict these interactions.

*Remember: Kinematics is the foundation of all motion in Physical AI - master this, and you'll understand how robots move! 💪*