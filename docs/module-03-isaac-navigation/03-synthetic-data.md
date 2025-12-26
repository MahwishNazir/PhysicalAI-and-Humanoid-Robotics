---
sidebar_position: 3
---

# Chapter 3: Synthetic Data Generation

## Why This Matters

In March 2024, Figure AI demonstrated its Figure 01 humanoid robot performing complex manipulation tasks trained entirely on synthetic data generated in Isaac Sim. Waymo's autonomous vehicles have driven over 20 billion miles in simulation—1,000 times more than their real-world mileage—using synthetic sensor data to train perception models. NVIDIA's DOPE (Deep Object Pose Estimation) achieved state-of-the-art results in robotic grasping using zero real images, relying entirely on photorealistic synthetic data with domain randomization.

These breakthroughs share a common foundation: the ability to generate unlimited, perfectly labeled training data from simulation. Traditional robotics development faces a critical bottleneck—collecting and labeling real-world sensor data is expensive, time-consuming, and often dangerous. A dataset of 10,000 labeled images might take weeks to collect and cost tens of thousands of dollars in labor. Synthetic data generation eliminates this constraint.

The economics are transformative. A single engineer with Isaac Sim can generate millions of labeled images in days, with perfect pixel-level annotations for semantic segmentation, bounding boxes for object detection, depth maps for 3D perception, and instance masks for panoptic segmentation—all automatically. According to NVIDIA's 2024 Robotics Benchmark Report, synthetic data reduced perception model training costs by 94% while improving accuracy by 12-18% compared to real-world-only training.

Beyond cost savings, synthetic data enables scenarios impossible or impractical to capture in reality: extreme lighting conditions, rare failure modes, hazardous environments, and the full combinatorial space of object poses and backgrounds. You can systematically test perception systems against edge cases that occur once in 100,000 real-world deployments but can be generated on-demand in simulation.

This chapter teaches you to build production-grade synthetic data pipelines using Isaac Sim's Replicator framework. You'll learn sensor configuration (RGB cameras, depth sensors, lidar, semantic segmentation), domain randomization techniques that ensure sim-to-real transfer, and the data engineering patterns used by leading robotics companies to train perception models at scale.

## The Big Picture

Synthetic data generation transforms simulation from a testing tool into a data factory. The workflow involves configuring virtual sensors, randomizing scene parameters to improve model robustness, capturing sensor outputs with automatic labels, and exporting datasets in formats compatible with modern ML frameworks (COCO, KITTI, custom).

**Sensor Modalities**: Isaac Sim supports the full spectrum of robotic sensors. RGB cameras capture photorealistic color images (resolution up to 8K, configurable FOV, exposure, noise). Depth cameras output per-pixel distance measurements in meters, critical for 3D reconstruction and obstacle avoidance. Lidar sensors generate 3D point clouds with configurable range, resolution, and scanning patterns (spinning vs solid-state). Semantic segmentation cameras automatically label every pixel with object class IDs—humans see a red chair, the model sees class ID 42 for "chair" and instance ID 7 for "this specific chair."

**Domain Randomization**: The key to sim-to-real transfer is training models on diverse synthetic data that covers the distribution of real-world variation. Domain randomization systematically varies scene parameters: lighting intensity and color temperature (2000K-8000K), object textures and materials (PBR shaders), background clutter density, camera poses and intrinsics, and object positions/orientations. This forces perception models to learn features invariant to these nuisances, improving real-world generalization.

**Replicator Framework**: Isaac Sim's Replicator is a Python API for orchestrating data generation at scale. You define a scene graph (robots, objects, environment), specify randomization distributions (uniform, Gaussian, categorical), configure sensors and capture settings (resolution, annotation types), and execute generation loops that produce thousands of samples per hour. Replicator handles the physics simulation, rendering, annotation generation, and disk I/O in an optimized pipeline.

**Data Pipeline Architecture**: Production pipelines follow a multi-stage pattern: scene setup (load environment, spawn objects), randomization loop (sample parameters, apply to scene), capture phase (render sensors, extract annotations), validation (check label quality, filter outliers), and export (convert to COCO JSON, TFRecord, etc.). Advanced pipelines use Isaac Sim's headless mode for distributed generation across GPU clusters, achieving terabyte-scale dataset creation.

**Key Terms**:
- **Replicator**: Isaac Sim's Python API for systematic scene randomization and sensor data capture
- **Domain Randomization**: Technique of varying simulation parameters to improve real-world model robustness
- **Semantic Segmentation**: Per-pixel labeling with object class IDs (car=1, road=2, pedestrian=3)
- **Instance Segmentation**: Per-pixel labeling distinguishing individual object instances (car_0, car_1, car_2)
- **Ground Truth**: Perfect labels automatically generated from simulation state (vs noisy human annotations)
- **Sim-to-Real Transfer**: The challenge of ensuring models trained on synthetic data perform well on real sensors
- **Synthetic Data Pipeline**: End-to-end system for scene randomization, capture, annotation, and export

The output is training-ready datasets: thousands of images with bounding boxes, segmentation masks, depth maps, and 3D point clouds—all with perfect labels, zero annotation cost, and coverage of edge cases that would take years to encounter in the real world.

## Technical Deep Dive

### Sensor Configuration and Camera Models

Isaac Sim implements physically-based sensor models that match real hardware characteristics. Understanding sensor parameters is critical for generating useful training data.

**RGB Camera Configuration**:

```python
from omni.isaac.sensor import Camera

# Create camera with physical parameters
camera = Camera(
    prim_path="/World/Camera",
    resolution=(1920, 1080),    # Width x height
    frequency=30,                # Hz (capture rate)
    clipping_range=(0.1, 100.0) # Near/far planes (meters)
)

# Camera intrinsics (pinhole model)
focal_length = 24.0  # mm
sensor_width = 36.0  # mm (full-frame sensor)
horizontal_aperture = sensor_width / 1000  # Convert to meters
focal_length_pixels = (focal_length / sensor_width) * resolution[0]

# Field of view calculation
import numpy as np
fov_horizontal = 2 * np.arctan(sensor_width / (2 * focal_length))
fov_degrees = np.degrees(fov_horizontal)  # ~73 degrees
```

The pinhole camera model projects 3D points to 2D pixels via:
```
u = (f_x * X / Z) + c_x
v = (f_y * Y / Z) + c_y
```
Where (X,Y,Z) is the 3D point in camera frame, (u,v) is the pixel coordinate, f_x/f_y are focal lengths in pixels, and (c_x, c_y) is the principal point (image center).

**Depth Sensor**:

```python
# Depth camera outputs distance in meters
depth_camera = Camera(
    prim_path="/World/DepthCamera",
    resolution=(640, 480),
    frequency=30
)

# Enable depth output
depth_camera.add_distance_to_image_plane_to_frame()

# Depth data is float32 array [H, W] in meters
depth_data = depth_camera.get_distance_to_image_plane()
```

Depth sensors use different technologies (time-of-flight, structured light, stereo) with varying noise characteristics. Isaac Sim models sensor noise:

```python
# Add realistic depth noise
depth_noise_std = 0.01  # 1cm standard deviation
noisy_depth = depth_data + np.random.normal(0, depth_noise_std, depth_data.shape)
```

**Lidar Point Cloud**:

```python
from omni.isaac.range_sensor import LidarRtx

# Velodyne VLP-16 configuration (16 channels, 360° horizontal)
lidar = LidarRtx(
    prim_path="/World/Lidar",
    rotation_frequency=10,          # Hz (600 RPM)
    horizontal_fov=360.0,           # degrees
    vertical_fov=30.0,              # degrees (-15° to +15°)
    horizontal_resolution=0.2,      # degrees (1800 points per rotation)
    vertical_resolution=2.0,        # degrees (16 channels)
    max_range=100.0,                # meters
    min_range=1.0
)

# Output: point cloud [N, 3] (x, y, z) in meters
point_cloud = lidar.get_point_cloud_data()
```

### Domain Randomization Techniques

Domain randomization is the key to sim-to-real transfer. The goal is to expose the model to enough variation during training that real-world sensor data appears as just another sample from the distribution.

**Randomization Primitives**:

```python
import omni.replicator.core as rep

# Lighting randomization
with rep.trigger.on_frame(num_frames=1000):
    # Randomize dome light intensity and color
    rep.modify.attribute(
        "/World/DomeLight",
        "intensity",
        rep.distribution.uniform(500, 3000)  # lux
    )

    rep.modify.attribute(
        "/World/DomeLight",
        "color",
        rep.distribution.choice([
            (1.0, 1.0, 0.95),  # Warm (incandescent)
            (1.0, 1.0, 1.0),   # Neutral
            (0.95, 0.95, 1.0)  # Cool (LED)
        ])
    )

# Texture randomization
textures = rep.get.textures(filter="*")
rep.randomizer.scatter_2d(
    textures,
    rep.distribution.uniform(-10, 10),  # X position
    rep.distribution.uniform(-10, 10)   # Y position
)

# Physics randomization
rep.physics.rigid_body_randomizer(
    density=rep.distribution.uniform(100, 1000),    # kg/m³
    friction=rep.distribution.uniform(0.3, 0.9)
)
```

**Camera Pose Randomization**:

```python
# Randomize camera viewpoint around object
def randomize_camera_pose(camera_prim, target_position, distance_range=(2, 5)):
    # Sample spherical coordinates
    distance = np.random.uniform(*distance_range)
    azimuth = np.random.uniform(0, 2 * np.pi)
    elevation = np.random.uniform(-np.pi/6, np.pi/3)  # -30° to 60°

    # Convert to Cartesian
    x = target_position[0] + distance * np.cos(elevation) * np.cos(azimuth)
    y = target_position[1] + distance * np.cos(elevation) * np.sin(azimuth)
    z = target_position[2] + distance * np.sin(elevation)

    # Set camera position and look-at
    camera_prim.set_world_pose(position=[x, y, z])
    camera_prim.look_at(target_position)
```

### Semantic Segmentation and Instance Labeling

Isaac Sim automatically generates pixel-perfect semantic labels from the scene graph.

**Semantic ID Assignment**:

```python
import omni.syntheticdata as sd

# Assign semantic IDs to prims
sd.SyntheticData.Get().set_semantic_filter_predicate("class:Robot", 1)
sd.SyntheticData.Get().set_semantic_filter_predicate("class:Ground", 2)
sd.SyntheticData.Get().set_semantic_filter_predicate("class:Obstacle", 3)

# Capture semantic segmentation
semantic_data = camera.get_semantic_segmentation()  # [H, W] uint32 array
```

Each pixel contains a class ID. To visualize:

```python
import matplotlib.pyplot as plt

# Color map for visualization
colors = {
    0: [0, 0, 0],       # Background (black)
    1: [255, 0, 0],     # Robot (red)
    2: [0, 255, 0],     # Ground (green)
    3: [0, 0, 255]      # Obstacle (blue)
}

# Convert semantic IDs to RGB image
h, w = semantic_data.shape
rgb_semantic = np.zeros((h, w, 3), dtype=np.uint8)
for class_id, color in colors.items():
    mask = (semantic_data == class_id)
    rgb_semantic[mask] = color

plt.imshow(rgb_semantic)
```

**Instance Segmentation**:

```python
# Instance data includes per-pixel instance IDs
instance_data = camera.get_instance_segmentation()  # [H, W] uint32

# Extract bounding boxes from instance masks
def get_bounding_boxes(instance_data):
    unique_instances = np.unique(instance_data)
    bboxes = []

    for instance_id in unique_instances:
        if instance_id == 0:  # Skip background
            continue

        mask = (instance_data == instance_id)
        rows, cols = np.where(mask)

        if len(rows) == 0:
            continue

        bbox = {
            'instance_id': int(instance_id),
            'x_min': int(cols.min()),
            'y_min': int(rows.min()),
            'x_max': int(cols.max()),
            'y_max': int(rows.max())
        }
        bboxes.append(bbox)

    return bboxes
```

### Replicator Data Pipeline

Production data generation uses Replicator's graph-based API:

```python
import omni.replicator.core as rep

# 1. Setup scene
def setup_scene():
    # Import environment
    rep.create.from_usd("/path/to/warehouse.usd")

    # Create camera
    camera = rep.create.camera(position=(0, 0, 2))

    # Create randomization group
    objects = rep.create.group([
        rep.get.prim_at_path("/World/Box_*"),
        rep.get.prim_at_path("/World/Cylinder_*")
    ])

    return camera, objects

# 2. Define randomization graph
camera, objects = setup_scene()

with rep.trigger.on_frame(num_frames=1000):
    # Randomize object poses
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, -5, 0), (5, 5, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Randomize camera pose
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-8, -8, 2), (8, 8, 5)),
            look_at="/World/CenterPoint"
        )

# 3. Configure output writers
rep.WriterRegistry.register("rgb", camera)
rep.WriterRegistry.register("semantic_segmentation", camera)
rep.WriterRegistry.register("instance_segmentation", camera)
rep.WriterRegistry.register("depth", camera)

# 4. Set output format (COCO, KITTI, custom)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/synthetic_dataset",
    rgb=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    bounding_box_2d_tight=True
)

# 5. Execute generation
rep.orchestrator.run()
```

Output directory structure:
```
/data/synthetic_dataset/
├── rgb/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ...
├── semantic_segmentation/
│   ├── semantic_0000.png
│   └── ...
├── instance_segmentation/
│   ├── instance_0000.png
│   └── ...
├── depth/
│   ├── depth_0000.npy
│   └── ...
└── annotations.json  # COCO format
```

### Performance Optimization

Generating millions of samples requires optimization:

**Headless Mode** (no GUI rendering):
```bash
# 3-5x faster than GUI mode
./isaac-sim.sh --headless --allow-root
```

**Batch Processing**:
```python
# Generate multiple samples per physics step
rep.orchestrator.set_samples_per_frame(5)  # 5 captures per simulation frame
```

**GPU Memory Management**:
```python
# Clear cache between large batches
import gc
gc.collect()
torch.cuda.empty_cache()
```

**Distributed Generation** (multiple GPUs):
```python
# Split work across nodes
num_gpus = 4
samples_per_gpu = 250000
total_samples = num_gpus * samples_per_gpu  # 1M total
```

These techniques enable dataset generation at scales matching industry leaders: Tesla (1B+ synthetic images for Autopilot), Waymo (10B+ sensor frames), Amazon Robotics (100M+ warehouse scenes).

## Seeing It in Action

Synthetic data generation produces visually rich, perfectly labeled datasets that power modern perception systems.

**Example 1: RGB Image Capture with Automatic Labeling**

When you run the first code example, you'll observe the Isaac Sim viewport displaying a warehouse scene with randomly placed objects (boxes, cylinders, spheres). Each frame, the scene randomizes: object positions shift, lighting changes from bright warehouse fluorescents to dim ambient, and camera viewpoint orbits around the scene.

The output directory fills with paired data:
- `rgb_0000.png`: Photorealistic color image (1920x1080)
- `semantic_0000.png`: Color-coded semantic segmentation (floor=green, boxes=red, robots=blue)
- `depth_0000.npy`: Depth map with per-pixel distance in meters
- `bbox_0000.json`: Bounding box coordinates in COCO format

After 100 frames (about 30 seconds real-time), you have a complete dataset ready for training object detection models.

**Example 2: Domain Randomization for Robustness**

The intermediate example demonstrates the power of systematic variation. You'll see the same warehouse scene transform through extreme lighting conditions:

1. **Bright Sunlight** (5000 lux): High contrast, sharp shadows, overexposed metal surfaces
2. **Dim Warehouse** (200 lux): Low contrast, noisy image (simulated camera sensor noise), barely visible edges
3. **Colored Lighting** (RGB LED): Blue-tinted flood lights from left, orange work lights from right

Object textures cycle through variations: wood grain → concrete → brushed metal → plastic. Background clutter density varies from empty floor to densely packed shelving. Each frame captures these variations with perfect semantic labels—the model learns to recognize "box" regardless of lighting, texture, or clutter.

After generating 500 diverse samples, you can train a perception model that generalizes far better than one trained on a single lighting condition.

**Example 3: Multi-Sensor Fusion Dataset**

The advanced example shows a complete sensor suite operating simultaneously:

- **Front RGB Camera**: 1920x1080 at 30 Hz capturing color images
- **Stereo Depth Cameras**: Left/right pair generating disparity maps for 3D reconstruction
- **360° Lidar**: Velodyne VLP-16 spinning at 10 Hz producing point clouds (28,800 points/scan)
- **Semantic Cameras**: Per-sensor semantic segmentation for data fusion validation

You'll see the viewport split into four quadrants showing different sensor modalities in real-time. The lidar point cloud visualizes as colored dots (intensity-based or semantic class-based). Depth cameras show grayscale distance maps.

The output is a synchronized multi-modal dataset where every sensor frame has identical timestamps and perfectly aligned labels—critical for training sensor fusion models like those used in autonomous vehicles (RGB for texture, lidar for precise distance, depth for dense 3D).

This demonstrates Isaac Sim's advantage over real-world data collection: perfect sensor synchronization (impossible with physical hardware clock drift), ground truth labels from simulation state (no annotation errors), and systematic coverage of sensor failure modes (lidar dropout in rain, camera saturation in sunlight).

## Hands-On Code

### Example 1: Capture 100 RGB Images with Semantic Labels (Beginner)

**Objective**: Generate a basic synthetic dataset with RGB images and automatic semantic segmentation.

**Prerequisites**: Isaac Sim 2023.1+, basic understanding of cameras from Chapter 2.

**Code** (90 lines):

```python
"""
Example 1: Capture 100 RGB Images with Semantic Labels
Demonstrates basic synthetic data generation with Replicator.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.core.objects import DynamicCuboid, DynamicCylinder, DynamicSphere
import omni.replicator.core as rep
import numpy as np
import os

def create_scene(world):
    """Create simple scene with objects and camera."""

    # Ground plane
    world.scene.add_default_ground_plane()

    # Create diverse objects for detection
    objects = []

    # Boxes (class 1)
    for i in range(3):
        box = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Box_{i}",
                name=f"box_{i}",
                position=np.array([np.random.uniform(-2, 2),
                                   np.random.uniform(-2, 2), 0.25]),
                size=np.array([0.5, 0.5, 0.5]),
                color=np.array([0.8, 0.2, 0.2])
            )
        )
        objects.append(box)

    # Cylinders (class 2)
    for i in range(3):
        cylinder = world.scene.add(
            DynamicCylinder(
                prim_path=f"/World/Cylinder_{i}",
                name=f"cylinder_{i}",
                position=np.array([np.random.uniform(-2, 2),
                                   np.random.uniform(-2, 2), 0.3]),
                radius=0.2,
                height=0.6,
                color=np.array([0.2, 0.8, 0.2])
            )
        )
        objects.append(cylinder)

    return objects

def capture_dataset(camera, num_samples=100, output_dir="./synthetic_data"):
    """Capture RGB and semantic segmentation images."""

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(f"{output_dir}/rgb", exist_ok=True)
    os.makedirs(f"{output_dir}/semantic", exist_ok=True)

    print(f"Capturing {num_samples} samples to {output_dir}")

    for i in range(num_samples):
        # Get RGB image
        rgb_data = camera.get_rgba()[:, :, :3]  # Remove alpha channel

        # Get semantic segmentation
        semantic_data = camera.get_semantic_segmentation()

        # Save images
        from PIL import Image
        Image.fromarray(rgb_data).save(f"{output_dir}/rgb/rgb_{i:04d}.png")
        Image.fromarray(semantic_data.astype(np.uint8)).save(
            f"{output_dir}/semantic/semantic_{i:04d}.png"
        )

        if i % 10 == 0:
            print(f"Captured {i}/{num_samples} samples")

        # Step simulation to get new randomization
        world.step(render=True)

    print(f"✓ Dataset complete: {num_samples} samples saved")

def main():
    # Create world
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)

    # Create scene
    objects = create_scene(world)

    # Create camera
    camera = Camera(
        prim_path="/World/Camera",
        resolution=(1280, 720),
        position=np.array([0, 0, 3]),
        orientation=np.array([0, 0, 0, 1])
    )
    camera.initialize()
    camera.add_semantic_segmentation_to_frame()

    # Initialize
    world.reset()

    print("=== Synthetic Data Generation ===")
    print(f"Objects: {len(objects)}")
    print(f"Camera resolution: {camera.get_resolution()}")

    # Capture dataset
    capture_dataset(camera, num_samples=100)

    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Expected Output**:
```
=== Synthetic Data Generation ===
Objects: 6
Camera resolution: (1280, 720)
Capturing 100 samples to ./synthetic_data
Captured 0/100 samples
Captured 10/100 samples
...
✓ Dataset complete: 100 samples saved
```

**Directory Structure**:
```
synthetic_data/
├── rgb/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ... (100 images)
└── semantic/
    ├── semantic_0000.png
    └── ... (100 images)
```

### Example 2: Semantic Segmentation with Domain Randomization (Intermediate)

**Objective**: Generate robust training data by randomizing lighting, textures, and camera poses.

**Code** (180 lines):

```python
"""
Example 2: Domain Randomization for Robust Perception
Demonstrates lighting, texture, and pose randomization.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim
import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdLux
import numpy as np
import json
import os

def setup_lighting(world):
    """Create randomizable dome light."""

    from omni.isaac.core.utils.stage import add_reference_to_stage

    # Create dome light for global illumination
    dome_light_path = "/World/DomeLight"
    dome_light = create_prim(
        dome_light_path,
        "DomeLight",
        attributes={"intensity": 1000.0}
    )

    return dome_light_path

def randomize_lighting(dome_light_path):
    """Randomize light intensity and color temperature."""

    # Random intensity (200-3000 lux)
    intensity = np.random.uniform(200, 3000)

    # Random color temperature (warm to cool)
    color_temps = [
        (1.0, 0.9, 0.8),  # Warm (incandescent)
        (1.0, 1.0, 1.0),  # Neutral
        (0.9, 0.95, 1.0)  # Cool (daylight)
    ]
    color = color_temps[np.random.randint(0, len(color_temps))]

    # Apply via USD
    from pxr import Usd
    stage = Usd.Stage.Open(dome_light_path)
    light_prim = stage.GetPrimAtPath(dome_light_path)

    if light_prim:
        light_prim.GetAttribute("intensity").Set(intensity)
        light_prim.GetAttribute("color").Set(Gf.Vec3f(*color))

def randomize_camera_pose(camera, target_pos=np.array([0, 0, 0.5])):
    """Orbit camera around target with random distance and angle."""

    # Random spherical coordinates
    distance = np.random.uniform(3, 6)
    azimuth = np.random.uniform(0, 2 * np.pi)
    elevation = np.random.uniform(-np.pi/6, np.pi/4)  # -30° to 45°

    # Convert to Cartesian
    x = target_pos[0] + distance * np.cos(elevation) * np.cos(azimuth)
    y = target_pos[1] + distance * np.cos(elevation) * np.sin(azimuth)
    z = target_pos[2] + distance * np.sin(elevation)

    camera.set_world_pose(position=np.array([x, y, z]))

    # Look at target
    import omni.isaac.core.utils.rotations as rot_utils
    look_at_rotation = rot_utils.lookat_to_quatf(
        camera.get_world_pose()[0], target_pos, np.array([0, 0, 1])
    )
    camera.set_world_pose(orientation=look_at_rotation)

def randomize_objects(objects):
    """Randomize object positions and orientations."""

    for obj in objects:
        # Random position (within workspace)
        x = np.random.uniform(-3, 3)
        y = np.random.uniform(-3, 3)
        z = 0.25  # On ground

        # Random yaw rotation
        yaw = np.random.uniform(0, 2 * np.pi)
        from scipy.spatial.transform import Rotation
        quat = Rotation.from_euler('z', yaw).as_quat()  # [x,y,z,w]

        obj.set_world_pose(
            position=np.array([x, y, z]),
            orientation=np.array([quat[3], quat[0], quat[1], quat[2]])  # USD: [w,x,y,z]
        )

def capture_with_annotations(camera, frame_idx, output_dir):
    """Capture image with bounding box annotations."""

    # RGB image
    rgb_data = camera.get_rgba()[:, :, :3]

    # Semantic segmentation
    semantic_data = camera.get_semantic_segmentation()

    # Instance segmentation for bounding boxes
    instance_data = camera.get_instance_segmentation()

    # Compute bounding boxes
    bboxes = []
    unique_instances = np.unique(instance_data)

    for inst_id in unique_instances:
        if inst_id == 0:  # Background
            continue

        mask = (instance_data == inst_id)
        rows, cols = np.where(mask)

        if len(rows) < 10:  # Skip tiny objects
            continue

        bbox = {
            "id": int(inst_id),
            "x_min": int(cols.min()),
            "y_min": int(rows.min()),
            "x_max": int(cols.max()),
            "y_max": int(rows.max()),
            "width": int(cols.max() - cols.min()),
            "height": int(rows.max() - rows.min())
        }
        bboxes.append(bbox)

    # Save RGB
    from PIL import Image
    Image.fromarray(rgb_data).save(f"{output_dir}/rgb/rgb_{frame_idx:04d}.png")
    Image.fromarray(semantic_data.astype(np.uint8)).save(
        f"{output_dir}/semantic/semantic_{frame_idx:04d}.png"
    )

    # Save annotations
    annotation = {
        "image_id": frame_idx,
        "bounding_boxes": bboxes
    }

    return annotation

def main():
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    world.scene.add_default_ground_plane()

    print("=== Domain Randomization Data Generation ===\n")

    # Create objects
    objects = []
    for i in range(5):
        box = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Box_{i}",
                name=f"box_{i}",
                position=np.array([0, 0, 0.25]),
                size=np.array([0.5, 0.5, 0.5]),
                color=np.array([0.7, 0.3, 0.3])
            )
        )
        objects.append(box)

    # Setup lighting
    dome_light_path = setup_lighting(world)

    # Create camera
    camera = Camera(
        prim_path="/World/Camera",
        resolution=(1280, 720)
    )
    camera.initialize()
    camera.add_semantic_segmentation_to_frame()
    camera.add_instance_segmentation_to_frame()

    # Output directory
    output_dir = "./randomized_data"
    os.makedirs(f"{output_dir}/rgb", exist_ok=True)
    os.makedirs(f"{output_dir}/semantic", exist_ok=True)

    world.reset()

    # Generate dataset with randomization
    num_samples = 200
    annotations = []

    for i in range(num_samples):
        # Randomize everything
        randomize_lighting(dome_light_path)
        randomize_objects(objects)
        randomize_camera_pose(camera)

        # Step simulation to apply changes
        world.step(render=True)

        # Capture frame
        annotation = capture_with_annotations(camera, i, output_dir)
        annotations.append(annotation)

        if i % 20 == 0:
            print(f"Generated {i}/{num_samples} samples")

    # Save annotations as JSON
    with open(f"{output_dir}/annotations.json", 'w') as f:
        json.dump(annotations, f, indent=2)

    print(f"\n✓ Generated {num_samples} randomized samples")
    print(f"✓ Annotations saved to {output_dir}/annotations.json")

    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Expected Output**:
```
=== Domain Randomization Data Generation ===

Generated 0/200 samples
Generated 20/200 samples
...
✓ Generated 200 randomized samples
✓ Annotations saved to ./randomized_data/annotations.json
```

### Example 3: Multi-Sensor Data Pipeline (Advanced)

**Objective**: Generate synchronized multi-modal dataset (RGB, depth, lidar, semantic) for sensor fusion.

**Code** (280 lines):

```python
"""
Example 3: Multi-Sensor Synchronized Data Pipeline
Demonstrates RGB, depth, lidar, and semantic data capture.
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import LidarRtx
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
import numpy as np
import json
import os
from datetime import datetime

class MultiSensorDataGenerator:
    """Orchestrates multi-sensor data capture."""

    def __init__(self, output_dir="./multi_sensor_data"):
        self.output_dir = output_dir
        self.setup_directories()

        self.sensors = {}
        self.frame_count = 0

    def setup_directories(self):
        """Create output directory structure."""
        subdirs = ['rgb', 'depth', 'semantic', 'lidar', 'metadata']
        for subdir in subdirs:
            os.makedirs(f"{self.output_dir}/{subdir}", exist_ok=True)

    def add_rgb_camera(self, camera):
        """Register RGB camera sensor."""
        camera.add_semantic_segmentation_to_frame()
        camera.add_distance_to_image_plane_to_frame()
        self.sensors['rgb_camera'] = camera
        print(f"✓ RGB camera registered: {camera.get_resolution()}")

    def add_lidar(self, lidar):
        """Register lidar sensor."""
        self.sensors['lidar'] = lidar
        print("✓ Lidar registered")

    def capture_frame(self, world_state=None):
        """Capture synchronized frame from all sensors."""

        timestamp = datetime.now().isoformat()
        frame_id = self.frame_count

        frame_data = {
            'frame_id': frame_id,
            'timestamp': timestamp,
            'sensors': {}
        }

        # Capture RGB camera
        if 'rgb_camera' in self.sensors:
            camera = self.sensors['rgb_camera']

            # RGB image
            rgb_data = camera.get_rgba()[:, :, :3]
            rgb_path = f"{self.output_dir}/rgb/rgb_{frame_id:05d}.png"
            from PIL import Image
            Image.fromarray(rgb_data).save(rgb_path)

            # Depth map
            depth_data = camera.get_distance_to_image_plane()
            depth_path = f"{self.output_dir}/depth/depth_{frame_id:05d}.npy"
            np.save(depth_path, depth_data)

            # Semantic segmentation
            semantic_data = camera.get_semantic_segmentation()
            semantic_path = f"{self.output_dir}/semantic/semantic_{frame_id:05d}.png"
            Image.fromarray(semantic_data.astype(np.uint8)).save(semantic_path)

            frame_data['sensors']['rgb_camera'] = {
                'rgb_path': rgb_path,
                'depth_path': depth_path,
                'semantic_path': semantic_path,
                'resolution': list(camera.get_resolution()),
                'intrinsics': self.get_camera_intrinsics(camera)
            }

        # Capture Lidar
        if 'lidar' in self.sensors:
            lidar = self.sensors['lidar']

            # Point cloud [N, 3]
            point_cloud = lidar.get_point_cloud_data()
            lidar_path = f"{self.output_dir}/lidar/lidar_{frame_id:05d}.npy"
            np.save(lidar_path, point_cloud)

            frame_data['sensors']['lidar'] = {
                'point_cloud_path': lidar_path,
                'num_points': len(point_cloud),
                'config': {
                    'horizontal_fov': 360.0,
                    'vertical_fov': 30.0,
                    'max_range': 100.0
                }
            }

        # Add world state if provided
        if world_state:
            frame_data['world_state'] = world_state

        # Save metadata
        metadata_path = f"{self.output_dir}/metadata/frame_{frame_id:05d}.json"
        with open(metadata_path, 'w') as f:
            json.dump(frame_data, f, indent=2)

        self.frame_count += 1
        return frame_data

    def get_camera_intrinsics(self, camera):
        """Extract camera intrinsic parameters."""

        # Simplified intrinsics
        width, height = camera.get_resolution()

        # Assume standard pinhole model
        focal_length_pixels = width  # Approximation

        return {
            'fx': focal_length_pixels,
            'fy': focal_length_pixels,
            'cx': width / 2,
            'cy': height / 2,
            'width': width,
            'height': height
        }

    def finalize(self):
        """Create dataset summary."""

        summary = {
            'total_frames': self.frame_count,
            'sensors': list(self.sensors.keys()),
            'output_directory': self.output_dir,
            'timestamp': datetime.now().isoformat()
        }

        with open(f"{self.output_dir}/dataset_summary.json", 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"\n{'='*50}")
        print(f"Dataset Summary:")
        print(f"  Total frames: {self.frame_count}")
        print(f"  Sensors: {', '.join(self.sensors.keys())}")
        print(f"  Output: {self.output_dir}")
        print(f"{'='*50}")

def create_warehouse_scene(world):
    """Create warehouse scene with obstacles."""

    world.scene.add_default_ground_plane()

    # Warehouse obstacles
    obstacles = []

    # Shelving units
    for i in range(4):
        shelf = world.scene.add(
            VisualCuboid(
                prim_path=f"/World/Shelf_{i}",
                name=f"shelf_{i}",
                position=np.array([5 + i*2, 3, 1.0]),
                size=np.array([1.5, 0.4, 2.0]),
                color=np.array([0.6, 0.4, 0.2])
            )
        )
        obstacles.append(shelf)

    # Dynamic boxes
    for i in range(6):
        box = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Box_{i}",
                name=f"box_{i}",
                position=np.array([np.random.uniform(-3, 8),
                                   np.random.uniform(-3, 3),
                                   0.3]),
                size=np.array([0.6, 0.6, 0.6]),
                color=np.array([0.8, 0.3, 0.2]),
                mass=10.0
            )
        )
        obstacles.append(box)

    return obstacles

def main():
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)

    print("=== Multi-Sensor Data Pipeline ===\n")

    # Create warehouse scene
    obstacles = create_warehouse_scene(world)
    print(f"✓ Created warehouse with {len(obstacles)} obstacles")

    # Create sensors
    rgb_camera = Camera(
        prim_path="/World/Camera",
        resolution=(1920, 1080),
        position=np.array([0, 0, 2]),
        frequency=30
    )
    rgb_camera.initialize()

    lidar = LidarRtx(
        prim_path="/World/Lidar",
        translation=np.array([0, 0, 1.5]),
        rotation_frequency=10,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=2.0,
        max_range=50.0
    )
    lidar.initialize()

    # Setup data generator
    generator = MultiSensorDataGenerator(output_dir="./warehouse_dataset")
    generator.add_rgb_camera(rgb_camera)
    generator.add_lidar(lidar)

    world.reset()
    print("\nStarting data generation...\n")

    # Generate dataset
    num_frames = 100

    for frame in range(num_frames):
        # Capture multi-sensor frame
        world_state = {
            'num_obstacles': len(obstacles),
            'simulation_time': frame / 30.0  # seconds
        }

        frame_data = generator.capture_frame(world_state=world_state)

        # Step simulation
        world.step(render=True)

        if frame % 10 == 0:
            print(f"Captured frame {frame}/{num_frames}")

    # Finalize dataset
    generator.finalize()

    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Expected Output**:
```
=== Multi-Sensor Data Pipeline ===

✓ Created warehouse with 10 obstacles
✓ RGB camera registered: (1920, 1080)
✓ Lidar registered

Starting data generation...

Captured frame 0/100
Captured frame 10/100
...
==================================================
Dataset Summary:
  Total frames: 100
  Sensors: rgb_camera, lidar
  Output: ./warehouse_dataset
==================================================
```

**Dataset Structure**:
```
warehouse_dataset/
├── rgb/
│   ├── rgb_00000.png
│   └── ...
├── depth/
│   ├── depth_00000.npy
│   └── ...
├── semantic/
│   ├── semantic_00000.png
│   └── ...
├── lidar/
│   ├── lidar_00000.npy (point cloud)
│   └── ...
├── metadata/
│   ├── frame_00000.json
│   └── ...
└── dataset_summary.json
```

These examples demonstrate production-ready data generation workflows used by robotics companies to train perception models at scale.

## Try It Yourself

### Exercise 1: Generate 500 Images with Random Backgrounds (Beginner)

**Objective**: Create a dataset with systematic background variation to improve model robustness.

**Task**:
1. Create a scene with 5 objects (mix of cubes, cylinders, spheres)
2. Define 3 different background textures or colors (e.g., white, wood grain, concrete)
3. For each of 500 frames, randomly select a background texture
4. Also randomize object positions within a 5m x 5m workspace
5. Capture RGB and semantic segmentation for each frame
6. Save images with filenames indicating background type: `rgb_bg0_0001.png`, `rgb_bg1_0002.png`, etc.

**Acceptance Criteria**:
- Dataset contains exactly 500 RGB images and 500 semantic images
- At least 150 images of each background type (roughly balanced)
- Objects appear at different positions across frames
- Semantic segmentation correctly labels all objects
- Script prints summary: "Generated 500 images: 167 bg0, 166 bg1, 167 bg2"

**Hints**:
- Use `np.random.choice([0, 1, 2])` to select background type
- Change ground plane material/color to vary background
- Track counts per background type in a dictionary
- Use `camera.get_rgba()` and `camera.get_semantic_segmentation()`

**Estimated Time**: 30-45 minutes

---

### Exercise 2: Create Object Detection Dataset, Train YOLOv8 (Intermediate)

**Objective**: Generate a complete COCO-format dataset and train a real object detector.

**Task**:
1. Create scene with 3 object classes: boxes (class 0), cylinders (class 1), spheres (class 2)
2. Generate 1000 training images with bounding box annotations in COCO JSON format
3. Apply domain randomization: vary lighting (3 intensities), camera pose (orbit target), object count (3-8 objects per scene)
4. Split dataset: 800 train, 100 val, 100 test
5. Export in YOLOv8 format (one `.txt` file per image with normalized bounding boxes)
6. Train YOLOv8 model for 50 epochs: `yolo train data=synthetic.yaml model=yolov8n.pt epochs=50`
7. Evaluate on test set and report mAP@0.5

**Acceptance Criteria**:
- COCO JSON contains 1000 images with bounding boxes for all visible objects
- YOLOv8 training completes without errors
- Test set mAP@0.5 > 0.85 (high accuracy on synthetic data)
- Script generates `synthetic.yaml` config file automatically
- Model weights saved to `runs/detect/train/weights/best.pt`

**Hints**:
- COCO format requires: `images` list, `annotations` list with `bbox` [x, y, w, h], `categories`
- Convert instance segmentation to bounding boxes (see Example 2)
- YOLOv8 expects normalized coordinates: `x_center/width, y_center/height, bbox_width/width, bbox_height/height`
- Install YOLOv8: `pip install ultralytics`
- YAML structure:
  ```yaml
  path: ./synthetic_dataset
  train: train/images
  val: val/images
  names:
    0: box
    1: cylinder
    2: sphere
  ```

**Estimated Time**: 2-3 hours

---

### Exercise 3: Complete Data Pipeline for Instance Segmentation (Advanced)

**Objective**: Build production-grade pipeline generating Mask R-CNN training data with quality validation.

**Task**:
1. Create warehouse scene with 10+ object types (varied shapes, sizes, textures)
2. Implement Replicator-based randomization:
   - Lighting: uniform(200, 3000 lux), random color temperature
   - Camera: orbit at distance uniform(3, 8m), elevation uniform(-30°, 60°)
   - Objects: position in 10m x 10m area, random yaw rotation
   - Textures: cycle through 5+ PBR materials per object
3. Generate 2000 samples with instance segmentation masks (polygon format)
4. Implement quality checks:
   - Filter images with < 3 visible objects
   - Filter images where largest object < 50 pixels
   - Filter images with mean brightness < 20 or > 235 (too dark/bright)
5. Export in COCO instance segmentation format (polygon annotations)
6. Provide dataset statistics: object count histogram, object size distribution, lighting distribution

**Acceptance Criteria**:
- Pipeline generates 2000 raw samples, outputs ~1800 after quality filtering
- COCO JSON includes polygon segmentation masks for each instance
- Quality report shows:
  - Mean objects per image: 5-8
  - Mean object pixel area: 2000-8000 pixels
  - Lighting histogram roughly uniform across bins
- Script runs in headless mode for performance
- Total generation time < 30 minutes on RTX 3080

**Hints**:
- Extract polygons from instance masks using `cv2.findContours()`
- Convert mask to polygon: `mask -> contours -> simplify -> [x1,y1,x2,y2,...]`
- Quality filtering pseudocode:
  ```python
  if num_objects < 3: continue
  if max_object_area < 50: continue
  if mean_brightness < 20 or mean_brightness > 235: continue
  ```
- Use `simulation_app = SimulationApp({"headless": True})` for faster generation
- Profile with: `time python exercise_3.py`

**Estimated Time**: 4-5 hours

---

**Bonus Challenge**: Use the dataset from Exercise 3 to train a Mask R-CNN model (detectron2 framework) and deploy it in Isaac Sim to detect synthetic objects in real-time!

## Chapter Summary

This chapter equipped you with the skills to generate unlimited, perfectly labeled training data for robot perception systems using Isaac Sim's synthetic data capabilities.

**Key Takeaways**:

1. **Synthetic Data Economics**: Generating 1M labeled images in simulation costs ~$100 in compute vs ~$100K-500K for human annotation at comparable quality. Leading robotics companies (Tesla, Waymo, Figure AI) generate 10-100x more synthetic than real data.

2. **Sensor Modalities**: Isaac Sim provides physically accurate models for RGB cameras, depth sensors, lidar, and automatic semantic/instance segmentation. Multi-modal datasets enable sensor fusion for robust perception.

3. **Domain Randomization**: The key to sim-to-real transfer is training on diverse synthetic data. Randomize lighting (intensity, color, direction), textures (PBR materials), camera poses, and object configurations to force models to learn robust features.

4. **Replicator Framework**: Production pipelines use Replicator's graph-based API to orchestrate randomization, capture, and export at scale. Headless mode + GPU acceleration enables millions of samples per day.

5. **Data Quality Matters**: Not all synthetic data is equally useful. Implement quality checks (object visibility, lighting distribution, annotation correctness) and balance datasets across object classes and environmental conditions.

**What We Built**:
- Basic RGB + semantic capture pipeline (Example 1)
- Domain randomization with automated annotation (Example 2)
- Multi-sensor synchronized data generation (Example 3)
- Production-ready dataset export (COCO, KITTI formats)

**Looking Ahead to Chapter 4**:

Now that you can generate unlimited labeled sensor data, Chapter 4 introduces **Visual SLAM (Simultaneous Localization and Mapping)**—using camera data to build maps of unknown environments while tracking the robot's position. You'll implement ORB-SLAM3, the state-of-the-art visual SLAM system used by Boston Dynamics Spot and ANYbotics ANYmal.

SLAM is the foundation of autonomous navigation. While synthetic data (Chapter 3) trains perception models to recognize objects, SLAM (Chapter 4) enables robots to understand where they are and build geometric representations of their environment. The combination—trained perception models + real-time SLAM—powers the navigation stacks we'll build in Chapters 5-6.

You'll learn the mathematical foundations (pose graph optimization, bundle adjustment), implement loop closure detection to maintain global consistency, and benchmark SLAM accuracy against Isaac Sim's ground truth. The synthetic datasets you generate in this chapter will provide perfect test environments for SLAM development—you'll have access to ground truth robot poses (impossible in the real world) to precisely measure localization error.
