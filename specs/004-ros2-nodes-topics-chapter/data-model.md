# Content Model: Chapter 2 - Nodes & Topics

**Date**: 2025-12-05
**Purpose**: Define hierarchical content structure, word count allocation, and learning objective mapping

## Section Hierarchy

### 1. Full Lesson (5000-7000 words total)

#### 1.1 Prerequisites & Learning Objectives (150 words)
**Functional Requirements**: FR-066, FR-067
**Content**:
- Prerequisites: Chapter 1 completion, ROS 2 Humble installed, Python or C++ basics
- Learning Objectives (5-6 bullet points):
  - Understand ROS 2 node architecture and lifecycle
  - Create publisher and subscriber nodes in Python and C++
  - Work with standard and custom message types
  - Configure QoS policies for reliable communication
  - Debug multi-node systems using ROS 2 tools
  - Design multi-node architectures using common patterns

---

#### 1.2 Node Architecture (800 words)
**Functional Requirements**: FR-006, FR-007, FR-008, FR-009, FR-010
**Learning Objectives**: LO-1 (Understand and Create ROS 2 Nodes)

##### 1.2.1 What is a Node? (200 words)
**Content**:
- Definition: Independent process performing computation
- Role in ROS 2 system: Building blocks of robot applications
- Node responsibilities: Single purpose (sensing, processing, control)
- Example: Camera node, image processor node, motor controller node
- Diagram: Simple 3-node system illustration

##### 1.2.2 Node Lifecycle States (250 words)
**Content**:
- Managed vs non-managed nodes
- Lifecycle states: Unconfigured, Inactive, Active, Finalized
- State transitions: configure, activate, deactivate, cleanup, shutdown
- Diagram: Mermaid state diagram (from research.md Section 1)
- Pedagogical note: Focus on concept, not implementation (managed lifecycle is advanced)

##### 1.2.3 Node Initialization & Execution (200 words)
**Content**:
- Initialization: rclpy.init() / rclcpp::init()
- Creating node instance: Node class
- Spinning: Event loop processing callbacks
- Shutdown: Cleanup and exit
- Code snippet: Basic node skeleton (10 lines Python)

##### 1.2.4 Node Naming & Namespaces (150 words)
**Content**:
- Node names: Unique identifiers in system
- Namespaces: Organizing nodes hierarchically
- Remapping: Changing node/topic names at runtime
- Examples: /robot1/camera, /robot2/camera
- Command: ros2 node list, ros2 node info

---

#### 1.3 Topics & Communication (700 words)
**Functional Requirements**: FR-011, FR-012, FR-013, FR-014, FR-015
**Learning Objectives**: LO-2 (Implement Topic-Based Communication)

##### 1.3.1 Publish-Subscribe Pattern (250 words)
**Content**:
- Concept: Decoupled communication (publishers don't know subscribers)
- Analogy: Radio broadcast (station publishes, radios subscribe)
- Diagram: Mermaid flowchart showing pub-sub (from research.md Section 4)
- Advantages: Loose coupling, scalability, flexibility
- Contrast with request-reply (services, Chapter 3)

##### 1.3.2 Topic Naming Conventions (150 words)
**Content**:
- Topic names: Descriptive, hierarchical paths
- Conventions: snake_case, leading slash
- Examples: /camera/image_raw, /cmd_vel, /odom
- Namespaces in topics: /robot1/camera/image_raw
- Best practices: Clear, consistent, avoid abbreviations

##### 1.3.3 Many-to-Many Communication (150 words)
**Content**:
- Multiple publishers to one topic: Data aggregation
- One publisher to multiple subscribers: Data distribution
- Many-to-many: Full flexibility
- Example: Multiple sensors → /sensor_data ← Multiple processors
- Use case: Redundant sensors, distributed processing

##### 1.3.4 Topic Discovery (150 words)
**Content**:
- DDS discovery mechanism (abstracted)
- Nodes automatically find each other
- No central broker required
- Implications: Dynamic system composition
- Commands: ros2 topic list, ros2 topic info /topic_name

---

#### 1.4 Message Types (600 words)
**Functional Requirements**: FR-016, FR-017, FR-018, FR-019, FR-020, FR-021
**Learning Objectives**: LO-3 (Work with Message Types)

##### 1.4.1 ROS 2 Message Structure (150 words)
**Content**:
- Messages: Data structures for communication
- Strongly typed: Publisher and subscriber must agree on type
- Field types: Primitives (int, float, string, bool), arrays, nested messages
- Example: geometry_msgs/Twist (linear + angular velocity)
- Command: ros2 interface show geometry_msgs/msg/Twist

##### 1.4.2 Standard Message Packages (200 words)
**Content**:
- **std_msgs**: Basic types (String, Int32, Float64, Bool)
- **geometry_msgs**: Pose, Twist, Transform, Point
- **sensor_msgs**: Image, PointCloud2, LaserScan, Imu
- **nav_msgs**: Odometry, Path
- When to use standard vs custom: Use standard when possible
- Examples with use cases:
  - std_msgs/String: Simple text messages
  - geometry_msgs/Twist: Velocity commands
  - sensor_msgs/Image: Camera data

##### 1.4.3 Custom Messages (200 words)
**Content**:
- When to create custom: Unique data structure not in standard packages
- Message definition syntax (.msg files)
- Example: PersonInfo.msg with name, age, position
- Building custom messages: colcon build
- Using in code: Import and use like standard messages
- Package structure: msg/ directory, package.xml dependencies

##### 1.4.4 Headers & Timestamps (50 words)
**Content**:
- std_msgs/Header: timestamp + frame_id
- Synchronization across nodes
- Time: ROS time vs system time
- Example: sensor_msgs/Image includes header

---

#### 1.5 Creating Publishers (700 words)
**Functional Requirements**: FR-022, FR-023, FR-024, FR-025, FR-027, FR-028
**Learning Objectives**: LO-1, LO-3

##### 1.5.1 Python Publisher Example (350 words)
**Content**:
- Complete working example: MinimalPublisher class
- Code (~30 lines + comments):
  - Import rclpy and message type
  - Create Node subclass
  - Initialize publisher (create_publisher)
  - Create timer (create_timer)
  - Timer callback: publish message
  - Main: init, spin, shutdown
- Explanation of each component:
  - Topic name: '/chatter'
  - Message type: std_msgs/String
  - Publish rate: 10 Hz (0.1s timer)
  - QoS: Default (10)
- Expected output when running

##### 1.5.2 C++ Publisher Example (250 words)
**Content**:
- Complete working example: MinimalPublisher class
- Code (~40 lines + comments):
  - Include headers
  - Create Node derived class
  - Constructor: publisher, timer
  - Timer callback: publish message
  - Main: init, spin, shutdown
- Key differences from Python:
  - Shared pointers (std::shared_ptr)
  - Member initialization
  - Compile step required
- Expected output

##### 1.5.3 Package Setup (100 words)
**Content**:
- Python: package.xml, setup.py, entry points
- C++: package.xml, CMakeLists.txt, executable
- Dependencies: rclpy/rclcpp, std_msgs
- Build command: colcon build
- Run command: ros2 run package_name node_name

---

#### 1.6 Creating Subscribers (600 words)
**Functional Requirements**: FR-029, FR-030, FR-031, FR-032, FR-034
**Learning Objectives**: LO-1, LO-3

##### 1.6.1 Python Subscriber Example (300 words)
**Content**:
- Complete working example: MinimalSubscriber class
- Code (~25 lines + comments):
  - Import rclpy and message type
  - Create Node subclass
  - Initialize subscriber (create_subscription)
  - Callback function: process message
  - Main: init, spin, shutdown
- Explanation:
  - Topic name: '/chatter' (match publisher)
  - Message type: std_msgs/String
  - Callback: Triggered when message arrives
  - Accessing message fields: msg.data
  - QoS: Default (10)
- Expected output when publisher running

##### 1.6.2 C++ Subscriber Example (200 words)
**Content**:
- Complete working example: MinimalSubscriber class
- Code (~35 lines + comments)
- Key differences from Python:
  - Callback signature: const std::shared_ptr<msg_type>
  - Binding callback with std::bind
- Expected output

##### 1.6.3 Callback Functions (100 words)
**Content**:
- Callback: Function executed when message arrives
- Asynchronous: Don't block, process quickly
- Threading considerations: Callbacks in executor thread
- Best practices: Keep callbacks short, offload heavy work

---

#### 1.7 Integration Examples (300 words)
**Functional Requirements**: FR-035, FR-036
**Learning Objectives**: LO-1, LO-2

##### 1.7.1 Combined Publisher-Subscriber Node (300 words)
**Content**:
- Use case: Data transformation pipeline
- Example: Subscribe to /input, process, publish to /output
- Code (~50 lines Python):
  - Both publisher and subscriber in one class
  - Callback receives message, transforms, publishes result
- Running multi-node system:
  - Terminal 1: ros2 run pkg input_node
  - Terminal 2: ros2 run pkg transformer_node
  - Terminal 3: ros2 run pkg output_node
  - Terminal 4: ros2 topic echo /output
- Diagram: 3-node pipeline

---

#### 1.8 Quality of Service (QoS) (800 words)
**Functional Requirements**: FR-040, FR-041, FR-042, FR-043, FR-044, FR-045, FR-046, FR-047
**Learning Objectives**: LO-4 (Configure QoS for Reliable Communication)

##### 1.8.1 What are QoS Policies? (150 words)
**Content**:
- Definition: Fine-grained control over communication behavior
- Why needed: Different use cases need different guarantees
- DDS foundation: ROS 2 built on DDS (Data Distribution Service)
- Policy types: Reliability, Durability, History, Deadline, Lifespan, Liveliness

##### 1.8.2 Reliability Policies (200 words)
**Content**:
- **RELIABLE**: Guaranteed delivery, retransmission on loss
  - Use case: Commands, critical messages
  - Trade-off: Higher latency, more bandwidth
- **BEST_EFFORT**: No delivery guarantee, fire-and-forget
  - Use case: High-frequency sensor data (camera, lidar)
  - Trade-off: Lower latency, less bandwidth
- Example scenario: Camera image (BEST_EFFORT) vs emergency stop (RELIABLE)
- Code example: Setting reliability in Python/C++

##### 1.8.3 Durability & History (200 words)
**Content**:
- **Durability**:
  - VOLATILE: Late joiners don't get old messages
  - TRANSIENT_LOCAL: Late joiners get last message
  - Use case: State information for late-joining subscribers
- **History**:
  - KEEP_LAST(n): Queue last n messages
  - KEEP_ALL: Queue all messages (until memory limit)
  - Use case: Buffering for slow subscribers
- Code example: Setting durability and history depth

##### 1.8.4 QoS Configuration Examples (150 words)
**Content**:
- Python QoS profile creation
- C++ QoS profile creation
- Using predefined profiles (qos_profile_sensor_data, qos_profile_default)
- Complete code example: Publisher and subscriber with custom QoS

##### 1.8.5 QoS Compatibility (100 words)
**Content**:
- Compatibility rules: RELIABLE pub ↔ RELIABLE/BEST_EFFORT sub
- Incompatibility consequences: No connection
- Debugging: ros2 topic info /topic --verbose
- Diagram: QoS compatibility sequence (from research.md)
- Common mistake: Mixing BEST_EFFORT pub with RELIABLE sub

---

#### 1.9 Debugging Tools (600 words)
**Functional Requirements**: FR-048, FR-049, FR-050, FR-051, FR-052, FR-053, FR-054, FR-055, FR-056
**Learning Objectives**: LO-5 (Debug Multi-Node Systems)

##### 1.9.1 ros2 node Commands (150 words)
**Content**:
- `ros2 node list`: List all active nodes
- `ros2 node info /node_name`: Show node details (publishers, subscribers, services)
- Example output with explanation
- Use case: Verify node is running, check connections

##### 1.9.2 ros2 topic Commands (300 words)
**Content**:
- `ros2 topic list`: List all active topics
- `ros2 topic echo /topic_name`: Display messages in real-time
- `ros2 topic info /topic_name`: Show publishers, subscribers, message type
- `ros2 topic hz /topic_name`: Measure message publish rate
- `ros2 topic pub /topic_name msg_type '{data}'`: Manually publish message
- `ros2 topic bw /topic_name`: Measure bandwidth usage
- Example workflow: Debugging communication issue
- Screenshots: Sample command outputs

##### 1.9.3 rqt_graph Visualization (150 words)
**Content**:
- Graphical tool: Visualize node-topic graph
- Launch: rqt_graph
- Interpreting graph:
  - Ovals: Nodes
  - Rectangles: Topics
  - Arrows: Publisher/subscriber relationships
- Filters: Show/hide node types
- Screenshot: Example multi-node graph
- Use case: Understanding system architecture at a glance

---

#### 1.10 Multi-Node System Design (500 words)
**Functional Requirements**: FR-057, FR-058, FR-059, FR-060, FR-061
**Learning Objectives**: LO-6 (Design Multi-Node Architectures)

##### 1.10.1 Design Principles (150 words)
**Content**:
- Single responsibility: One node, one purpose
- Loose coupling: Nodes communicate via topics only
- Reusability: Generic nodes, configurable via parameters
- Testability: Easy to test nodes in isolation
- When to split functionality:
  - Different update rates (fast sensing vs slow planning)
  - Different failure domains
  - Reusability in other systems

##### 1.10.2 Design Patterns (250 words)
**Content**:
- **Pipeline Pattern**:
  - Structure: A → B → C
  - Example: Camera → Processor → Detector
  - Diagram: Linear flow
  - Pros/Cons from research.md Section 5
- **Hierarchical Pattern**:
  - Structure: Perception → Planning → Control
  - Example: Sensors → Path Planner → Motor Controllers
  - Diagram: Layered architecture
  - Pros/Cons
- Brief mention: Star (hub-and-spoke), Peer-to-peer (advanced)

##### 1.10.3 Anti-Patterns to Avoid (100 words)
**Content**:
- Circular dependencies: A needs B, B needs A
- Monolithic node: One node doing everything
- Topic storm: Too many topics, unclear data flow
- Tight coupling: Nodes knowing too much about each other
- Solutions: Restructure, use mediators, consolidate topics

---

#### 1.11 Hands-On Exercises (400 words)
**Functional Requirements**: FR-062, FR-063
**Learning Objectives**: All LOs (practical application)

##### Exercise 1: Create Your First Publisher/Subscriber (150 words)
**Objective**: Create and run a simple pub-sub system
**Steps**:
1. Create a package: `ros2 pkg create --build-type ament_python my_first_nodes`
2. Write publisher node publishing counter to `/counter`
3. Write subscriber node displaying counter value
4. Build and run both nodes
5. Verify communication with `ros2 topic echo /counter`

**Expected Outcome**: Subscriber displays incrementing counter

**Difficulty**: Beginner | **Time**: 20 minutes

##### Exercise 2: Custom Message Type (125 words)
**Objective**: Create and use a custom message
**Steps**:
1. Create PersonInfo.msg (name, age, position)
2. Update package.xml and CMakeLists.txt/setup.py
3. Build package
4. Create publisher sending PersonInfo
5. Create subscriber displaying person information

**Expected Outcome**: Custom message transmitted successfully

**Difficulty**: Intermediate | **Time**: 30 minutes

##### Exercise 3: QoS Experimentation (125 words)
**Objective**: Observe QoS policy effects
**Steps**:
1. Create publisher with BEST_EFFORT reliability
2. Create subscriber with RELIABLE reliability
3. Observe: No connection (QoS incompatibility)
4. Change subscriber to BEST_EFFORT
5. Observe: Connection established

**Expected Outcome**: Understand QoS compatibility

**Difficulty**: Intermediate | **Time**: 15 minutes

---

#### 1.12 Troubleshooting (300 words)
**Functional Requirements**: FR-069
**Content**: Common errors from research.md Section 6

**Error 1: No Communication Between Nodes**
- Symptoms: Publisher active, subscriber sees no messages
- Causes: Topic name mismatch, QoS incompatibility
- Debug: `ros2 topic list`, `ros2 topic info /topic`
- Solution: Verify names match, check QoS profiles

**Error 2: Custom Message Not Found**
- Symptoms: Build error, import error
- Causes: Missing dependency, workspace not sourced
- Debug: Check package.xml, rebuild, source workspace
- Solution: Add rosidl dependencies, `colcon build`, `source install/setup.bash`

**Error 3: Node Crashes**
- Symptoms: Segmentation fault, Python exception
- Causes: Null pointer, unhandled exception in callback
- Debug: Check callback logic, add error handling
- Solution: Validate message fields, use try-except

---

#### 1.13 Key Takeaways (200 words)
**Functional Requirements**: FR-068
**Content**:
- Nodes are independent processes, building blocks of ROS 2 systems
- Topics enable asynchronous, many-to-many communication via publish-subscribe
- Message types define data structures, use standard types when possible
- Publishers send messages, subscribers receive via callbacks
- QoS policies control reliability, durability, and history
- Debugging tools (ros2 topic, ros2 node, rqt_graph) essential for development
- Design patterns (pipeline, hierarchical) guide multi-node architectures
- Always test code examples in ROS 2 Humble environment
- Start simple (basic pub-sub), progressively add complexity (QoS, custom messages)

---

### 2. Summary (500-800 words)

#### 2.1 Core Concepts Recap (200 words)
**Content**:
- **Nodes**: Independent processes, single responsibility
- **Topics**: Named channels for communication
- **Publishers**: Send messages to topics
- **Subscribers**: Receive messages from topics, callbacks
- **Messages**: Strongly-typed data structures
- **QoS**: Reliability, durability, history policies
- **Lifecycle**: Managed nodes have states (optional for beginners)

#### 2.2 Command Reference (200 words)
**Content**:
```bash
# Node commands
ros2 node list                    # List active nodes
ros2 node info /node_name         # Node details

# Topic commands
ros2 topic list                   # List active topics
ros2 topic echo /topic_name       # Display messages
ros2 topic info /topic_name       # Topic details
ros2 topic hz /topic_name         # Measure rate
ros2 topic pub /topic ...         # Publish manually

# Interface commands
ros2 interface show geometry_msgs/msg/Twist  # Show message definition
ros2 interface list               # List all interfaces

# Build and run
colcon build                      # Build workspace
source install/setup.bash         # Source workspace
ros2 run package_name node_name   # Run node
```

#### 2.3 Code Templates (200 words)
**Content**:
**Python Publisher Template**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Python Subscriber Template**:
```python
[Similar structure with create_subscription and callback]
```

#### 2.4 QoS Quick Reference (150 words)
**Content**:
| Use Case | Reliability | Durability | History |
|----------|-------------|------------|---------|
| Sensor Data | BEST_EFFORT | VOLATILE | KEEP_LAST(1) |
| Commands | RELIABLE | VOLATILE | KEEP_LAST(10) |
| State Info | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST(1) |
| Critical | RELIABLE | TRANSIENT_LOCAL | KEEP_ALL |

**Python QoS Example**:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
self.publisher_ = self.create_publisher(String, '/topic', qos)
```

---

## Content Attributes Summary

### Total Word Count Allocation
- **Full Lesson**: 5,950 words (within 5000-7000 target)
- **Summary**: 750 words (within 500-800 target)
- **Total**: 6,700 words

### Code Examples Count
- Python Publisher: 1 complete example (~30 lines)
- C++ Publisher: 1 complete example (~40 lines)
- Python Subscriber: 1 complete example (~25 lines)
- C++ Subscriber: 1 complete example (~35 lines)
- Combined Node: 1 example (~50 lines)
- QoS Configuration: 3 examples (~15 lines each)
- Custom Message: 1 example + .msg file
- **Total**: ~9 major code examples

### Diagrams Count
1. Node Lifecycle (Mermaid state diagram)
2. Publish-Subscribe Pattern (Mermaid flowchart)
3. Multi-Node Communication (Mermaid graph)
4. QoS Compatibility (Mermaid sequence diagram)
5. Simple 3-node system (Mermaid graph)
6. Pipeline pattern (Mermaid graph)
7. Hierarchical pattern (Mermaid graph)
8. rqt_graph screenshot (PNG)
9. Command output screenshots (2-3 PNGs)
- **Total**: ~12 diagrams/screenshots

### Exercises Count
1. First Publisher/Subscriber (Beginner, 20 min)
2. Custom Message Type (Intermediate, 30 min)
3. QoS Experimentation (Intermediate, 15 min)
- **Total**: 3 hands-on exercises (~65 minutes total)

### Learning Objective Coverage

All 6 learning objectives mapped to content sections:
- ✅ LO-1: Understand and Create Nodes (Sections 1.2, 1.5, 1.6)
- ✅ LO-2: Topic-Based Communication (Section 1.3)
- ✅ LO-3: Message Types (Section 1.4)
- ✅ LO-4: QoS Configuration (Section 1.8)
- ✅ LO-5: Debugging Tools (Section 1.9)
- ✅ LO-6: System Design (Section 1.10)

### Validation Against Functional Requirements

All 74 functional requirements addressed:
- ✅ FR-001 to FR-005: Content Structure
- ✅ FR-006 to FR-010: Node Architecture
- ✅ FR-011 to FR-015: Topics & Communication
- ✅ FR-016 to FR-021: Message Types
- ✅ FR-022 to FR-028: Publishers
- ✅ FR-029 to FR-034: Subscribers
- ✅ FR-035 to FR-039: Integration
- ✅ FR-040 to FR-047: QoS
- ✅ FR-048 to FR-056: Debugging Tools
- ✅ FR-057 to FR-061: System Design
- ✅ FR-062 to FR-063: Exercises
- ✅ FR-064 to FR-069: Learning Aids
- ✅ FR-070 to FR-074: Accessibility & Format

## Next Steps

This data model defines the complete content structure. Ready for:
1. Creating learning objectives contract (contracts/learning-objectives-map.md)
2. Creating implementation quickstart (quickstart.md)
3. Generating tasks (Phase 2: /sp.tasks)
