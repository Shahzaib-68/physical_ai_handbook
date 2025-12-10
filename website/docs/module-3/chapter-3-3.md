---
sidebar_position: 13
slug: /module-3/chapter-3-3
title: Chapter 3.3 - Synthetic Data Generation
---

# Chapter 3.3: Synthetic Data Generation

## Overview

Synthetic data generation is a transformative approach in robotics and AI that leverages simulation environments to create labeled training datasets without the need for real-world data collection. This chapter explores techniques for generating high-quality synthetic data using simulation platforms like Isaac Sim and Gazebo, focusing on applications for perception, control, and decision-making systems.

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the principles and benefits of synthetic data generation
- Implement synthetic data generation pipelines for robotics applications
- Configure realistic sensor models to produce synthetic sensor data
- Apply domain randomization techniques to improve transfer to reality
- Generate diverse training datasets with proper annotations
- Validate synthetic data quality against real-world data
- Design synthetic data systems that bridge the reality gap

## Introduction to Synthetic Data Generation

Synthetic data generation involves creating artificial datasets using computer simulations rather than collecting data from the real world. In robotics, this approach is particularly valuable because:

### Benefits of Synthetic Data:
- **Cost-effective**: No need for expensive data collection campaigns
- **Scalable**: Generate unlimited data samples
- **Safe**: Train on dangerous scenarios without risk
- **Controlled**: Perfect ground truth annotations
- **Diverse**: Systematically vary environmental conditions
- **Fast**: Accelerate development cycles

### Types of Synthetic Data for Robotics:
1. **Visual Data**: RGB images, depth maps, semantic segmentation
2. **Sensor Data**: LiDAR point clouds, IMU readings, force/torque
3. **Multi-modal Data**: Combinations of different sensor types
4. **Temporal Data**: Sequences for dynamic models
5. **Action Data**: Robot actions and their consequences

## Synthetic Data Generation Pipeline

### Components of a Synthetic Data Pipeline:

```
[Environment Setup] → [Domain Randomization] → [Sensor Simulation] → [Annotation Generation] → [Data Storage] → [Quality Validation]
```

### 1. Environment Setup:
- Define scene geometry and objects
- Configure physics properties
- Set up lighting and materials

### 2. Domain Randomization:
- Vary environmental parameters
- Randomize object properties
- Change lighting conditions

### 3. Sensor Simulation:
- Configure realistic sensors
- Apply noise models
- Generate sensor-specific outputs

### 4. Annotation Generation:
- Generate ground truth labels
- Create bounding boxes, segmentation masks
- Calculate 3D poses

### 5. Data Storage:
- Organize data in standard formats
- Include metadata and annotations
- Implement efficient storage systems

### 6. Quality Validation:
- Compare with real data distributions
- Validate model performance on real data
- Assess reality transfer

## Generating Visual Data

### RGB Images with Annotations:

```python
# Example synthetic data generation using Isaac Sim
from omni.isaac.kit import SimulationApp
import omni.isaac.core.utils.nucleus as nucleus_utils
from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.synthetic_utils import visualize
import numpy as np
import cv2
import json

# Start simulation
simulation_app = SimulationApp({"headless": False})

# Initialize world
world = World(stage_units_in_meters=1.0)

# Create camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.0, 0.0, 1.0]),
    frequency=30,
    resolution=(640, 480)
)

# Add domain randomization elements
def randomize_scene():
    # Change material properties
    # Move objects to random positions
    # Adjust lighting conditions
    pass

def generate_dataset(num_samples=1000):
    dataset = []
    
    for i in range(num_samples):
        # Randomize scene
        randomize_scene()
        
        # Capture RGB image
        rgb_data = camera.get_rgb()
        
        # Generate annotations
        depth_data = camera.get_depth()
        seg_data = camera.get_semantic_segmentation()
        bbox_data = camera.get_bounding_boxes()
        
        # Save image
        image_path = f"dataset/images/sample_{i:04d}.png"
        cv2.imwrite(image_path, cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))
        
        # Save annotations
        annotation = {
            "image_path": image_path,
            "depth_path": f"dataset/depth/sample_{i:04d}.png",
            "seg_path": f"dataset/segmentation/sample_{i:04d}.png",
            "bbox_path": f"dataset/bboxes/sample_{i:04d}.json",
            "objects": []
        }
        
        # Add object information
        for bbox in bbox_data:
            annotation["objects"].append({
                "class": bbox["class"],
                "bbox": [int(x) for x in bbox["bbox"]],
                "pose": bbox["pose"]
            })
        
        dataset.append(annotation)
        
        # Save annotation file
        with open(f"dataset/annotations/sample_{i:04d}.json", 'w') as f:
            json.dump(annotation, f)
        
        # Reset for next iteration
        world.step(render=True)
    
    # Save dataset info
    with open("dataset/dataset_info.json", 'w') as f:
        json.dump({"total_samples": num_samples}, f)

# Generate dataset
generate_dataset(100)

simulation_app.close()
```

## Domain Randomization Techniques

Domain randomization helps bridge the reality gap by training models on diverse synthetic data:

### 1. Texture Randomization:
```python
def randomize_textures():
    # Randomize surface textures
    textures = ["wood", "metal", "plastic", "concrete", "fabric"]
    selected_texture = np.random.choice(textures)
    
    # Apply to objects
    for obj_path in get_all_object_paths():
        apply_texture(obj_path, f"textures/{selected_texture}.png")
```

### 2. Lighting Randomization:
```python
def randomize_lighting():
    # Randomize dome light
    dome_light = get_prim_at_path("/World/DomeLight")
    dome_light.GetAttribute("inputs:intensity").Set(np.random.uniform(50, 500))
    dome_light.GetAttribute("inputs:color").Set(
        (np.random.uniform(0.8, 1.0), 
         np.random.uniform(0.8, 1.0), 
         np.random.uniform(0.8, 1.0))
    )
    
    # Randomize directional lights
    for i in range(3):  # 3 directional lights
        light_path = f"/World/DirectionalLight_{i}"
        light = get_prim_at_path(light_path)
        
        # Randomize position
        light.GetAttribute("xformOp:translate").Set(
            [np.random.uniform(-5, 5), 
             np.random.uniform(-5, 5), 
             np.random.uniform(5, 10)]
        )
        
        # Randomize intensity and color
        light.GetAttribute("inputs:intensity").Set(np.random.uniform(200, 1000))
        light.GetAttribute("inputs:color").Set(
            (np.random.uniform(0.7, 1.0), 
             np.random.uniform(0.7, 1.0), 
             np.random.uniform(0.7, 1.0))
        )
```

### 3. Object Pose Randomization:
```python
def randomize_object_poses():
    # Get all objects in the scene
    obj_paths = get_all_object_paths()
    
    for obj_path in obj_paths:
        # Random position
        new_pos = [
            np.random.uniform(-2.0, 2.0),  # x
            np.random.uniform(-2.0, 2.0),  # y
            np.random.uniform(0.1, 2.0)    # z
        ]
        
        # Random orientation
        new_rot = [
            np.random.uniform(-1.0, 1.0),  # x
            np.random.uniform(-1.0, 1.0),  # y
            np.random.uniform(-1.0, 1.0),  # z
            np.random.uniform(-1.0, 1.0)   # w
        ]
        
        # Normalize quaternion
        new_rot = np.array(new_rot) / np.linalg.norm(new_rot)
        
        # Apply transformation
        set_object_transform(obj_path, new_pos, new_rot)
```

## Sensor-Specific Synthetic Data Generation

### LiDAR Point Cloud Generation:
```python
from omni.isaac.sensor import RotatingLidarSensor
import numpy as np

def generate_lidar_dataset(num_samples=1000):
    # Create LiDAR sensor
    lidar = RotatingLidarSensor(
        prim_path="/World/Lidar",
        position=np.array([0.0, 0.0, 0.5]),
        name="front_lidar",
        fov=360,
        horizontal_resolution=0.25,
        vertical_resolution=2.0,
        height=16,
        rotation_frequency=10,
        samples_per_scan=360*4,
        max_range=25.0
    )
    
    for i in range(num_samples):
        # Randomize scene
        randomize_scene()
        
        # Capture LiDAR data
        lidar_data = lidar.get_point_cloud()
        
        # Apply noise
        if apply_noise:
            noise = np.random.normal(0, 0.01, lidar_data.shape)
            lidar_data += noise
        
        # Save point cloud
        pointcloud_path = f"dataset/lidar/pointcloud_{i:04d}.npy"
        np.save(pointcloud_path, lidar_data)
        
        # Generate ground truth for objects
        objects_gt = get_object_positions()
        with open(f"dataset/lidar/gt_{i:04d}.json", 'w') as f:
            json.dump(objects_gt, f)
```

### IMU Data Generation:
```python
from omni.isaac.sensor import ImuSensor

def generate_imu_dataset(num_samples=1000, sequence_length=100):
    imu = ImuSensor(
        prim_path="/World/Robot/Imu",
        position=np.array([0.0, 0.0, 0.0]),
        name="robot_imu",
        frequency=100
    )
    
    # Define robot motion sequences
    motion_sequences = [
        "standing", "walking_forward", "turning", "walking_sideways"
    ]
    
    for i in range(num_samples):
        # Select random motion
        motion = np.random.choice(motion_sequences)
        
        # Execute motion and collect IMU data
        sequence_data = []
        for step in range(sequence_length):
            # Execute motion command
            execute_motion_command(motion, step)
            
            # Get IMU readings
            imu_reading = {
                "linear_acceleration": imu.get_linear_acceleration(),
                "angular_velocity": imu.get_angular_velocity(),
                "orientation": imu.get_orientation()
            }
            
            # Add realistic noise
            imu_reading = add_imu_noise(imu_reading)
            
            sequence_data.append(imu_reading)
            
            # Step simulation
            world.step(render=False)
        
        # Save sequence
        np.save(f"dataset/imu/sequence_{i:04d}.npy", np.array(sequence_data))
        
        # Save labels
        with open(f"dataset/imu/labels_{i:04d}.txt", 'w') as f:
            f.write(motion)
```

## Multi-Modal Data Generation

Creating datasets that combine multiple sensor modalities:

```python
def generate_multimodal_dataset():
    # Setup multiple sensors
    camera = setup_camera()
    lidar = setup_lidar()
    imu = setup_imu()
    
    # Synchronize sensor capture
    def capture_synchronized_data():
        # Capture all sensors at the same time
        rgb_data = camera.get_rgb()
        depth_data = camera.get_depth()
        lidar_data = lidar.get_point_cloud()
        imu_data = imu.get_readings()
        
        # Package into single sample
        sample = {
            "rgb": rgb_data,
            "depth": depth_data,
            "lidar": lidar_data,
            "imu": imu_data,
            "timestamp": get_current_timestamp(),
            "annotations": generate_annotations()
        }
        
        return sample
    
    # Generate dataset
    dataset = []
    for i in range(1000):  # Number of samples
        # Randomize environment
        randomize_scene()
        
        # Capture synchronized data
        sample = capture_synchronized_data()
        
        # Save sample
        save_multimodal_sample(sample, i)
        dataset.append(sample)
    
    return dataset
```

## Annotation Generation

### Semantic Segmentation:
```python
def generate_semantic_segmentation():
    # Enable semantic segmentation in camera
    camera.add_modifiers([{
        "name": "annotate_semantic_segmentation",
        "enabled": True
    }])
    
    # Get semantic segmentation data
    seg_data = camera.get_semantic_segmentation()
    
    # Convert to class labels
    # Map semantic IDs to class names
    class_mapping = {
        1: "robot",
        2: "object",
        3: "ground",
        4: "wall"
    }
    
    # Create segmentation mask
    seg_mask = np.zeros_like(seg_data, dtype=np.uint8)
    for semantic_id, class_id in class_mapping.items():
        seg_mask[seg_data == semantic_id] = class_id
    
    return seg_mask
```

### 3D Object Pose Estimation:
```python
def generate_6d_pose_annotations():
    # Get object poses in simulation
    object_poses = {}
    for obj_path in get_all_object_paths():
        prim = get_prim_at_path(obj_path)
        position = prim.GetAttribute("xformOp:translate").Get()
        orientation = prim.GetAttribute("xformOp:orient").Get()
        
        object_poses[obj_path] = {
            "position": position,
            "orientation": orientation,
            "class": get_object_class(obj_path)
        }
    
    return object_poses
```

## Quality Assessment and Validation

### Statistical Comparison:
```python
def validate_synthetic_data_quality(real_data_path, synthetic_data_path):
    # Load real and synthetic data
    real_data = load_data(real_data_path)
    synthetic_data = load_data(synthetic_data_path)
    
    # Compare statistical properties
    real_mean = np.mean(real_data)
    synth_mean = np.mean(synthetic_data)
    
    real_std = np.std(real_data)
    synth_std = np.std(synthetic_data)
    
    # Calculate statistical distances
    mean_diff = abs(real_mean - synth_mean)
    std_diff = abs(real_std - synth_std)
    
    # Kolmogorov-Smirnov test
    ks_statistic, p_value = ks_2samp(real_data.flatten(), synthetic_data.flatten())
    
    # Return quality metrics
    quality_metrics = {
        "mean_diff": mean_diff,
        "std_diff": std_diff,
        "ks_statistic": ks_statistic,
        "p_value": p_value,
        "pass_validation": p_value > 0.05  # If p-value > 0.05, distributions are similar
    }
    
    return quality_metrics
```

### Model Performance Validation:
```python
def validate_model_transfer():
    # Train model on synthetic data
    synth_model = train_model_on_synthetic_data()
    
    # Evaluate on real data
    real_accuracy = evaluate_model_on_real_data(synth_model)
    
    # Train model on real data (if available)
    real_model = train_model_on_real_data()
    baseline_accuracy = evaluate_model_on_real_data(real_model)
    
    # Calculate transfer efficiency
    transfer_efficiency = real_accuracy / baseline_accuracy
    
    return {
        "synthetic_accuracy": real_accuracy,  # When evaluated on real data
        "baseline_accuracy": baseline_accuracy,
        "transfer_efficiency": transfer_efficiency
    }
```

## Best Practices for Synthetic Data Generation

### 1. Diversity and Coverage:
- Ensure synthetic data covers the full range of operational conditions
- Include edge cases and rare scenarios
- Systematically vary environmental parameters

### 2. Realism vs. Diversity Trade-off:
- Balance photorealism with variation
- Prioritize domain variation over visual perfection
- Focus on physically plausible rather than visually perfect

### 3. Annotation Quality:
- Ensure accurate ground truth generation
- Use multiple annotation methods for validation
- Include uncertainty estimates when possible

### 4. Validation Framework:
- Establish clear validation metrics
- Compare synthetic and real data distributions
- Test model performance on real-world tasks

### 5. Computational Efficiency:
- Optimize simulation performance
- Use parallel processing for data generation
- Implement efficient data storage and retrieval

## Synthetic Data Formats

### COCO Format for Object Detection:
```json
{
  "info": {
    "year": 2023,
    "version": "1.0",
    "description": "Synthetic Object Detection Dataset",
  },
  "images": [
    {
      "id": 1,
      "file_name": "image_0001.png",
      "height": 480,
      "width": 640
    }
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [100, 150, 50, 60],
      "area": 3000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {
      "id": 1,
      "name": "robot"
    }
  ]
}
```

### KITTI Format for 3D Detection:
```
# Each line: type, truncation, occlusion, alpha, bbox, dimensions, location, rotation_y, score
Car -1 -1 1.86 [656.35 175.2 691.05 225.91] 1.70 1.97 4.25 -1.59 1.72 2.45 15.78 0.01
```

## Bridging the Reality Gap

### Techniques to Improve Transfer:
1. **Progressive Domain Randomization**: Start with realistic conditions and gradually increase variation
2. **Adversarial Domain Adaptation**: Use GANs to make synthetic data look more realistic
3. **Sim-to-Real Transfer Learning**: Use synthetic pre-trained models
4. **Data Augmentation**: Apply realistic augmentations to synthetic data

### Validation Strategies:
- Test on diverse real-world scenarios
- Use domain adaptation techniques
- Implement gradual transfer protocols

## Summary

Synthetic data generation is a powerful approach for developing robust robotic systems that can operate effectively in the real world. By leveraging simulation environments with photorealistic rendering and accurate physics, we can create diverse, annotated datasets that are essential for training modern AI models.

The key to success lies in balancing the realism of synthetic data with the diversity needed for robust model training. Domain randomization, proper validation against real-world data, and careful attention to sensor noise models are critical for creating synthetic datasets that truly accelerate robotics development.

## Exercises

1. Generate a synthetic dataset for object detection using domain randomization techniques.
2. Create a multimodal dataset combining RGB images, depth maps, and LiDAR point clouds.
3. Implement a systematic validation framework to assess synthetic data quality.
4. Apply synthetic data to train a perception model and evaluate its performance on real data.
5. Design a domain randomization system that systematically varies relevant environmental conditions.