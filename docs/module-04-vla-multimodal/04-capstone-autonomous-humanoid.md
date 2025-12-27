# Chapter 4: Autonomous Humanoid Capstone Project

## Project Overview

Welcome to the capstone project of Module 4. Over the previous three chapters, you've learned the foundational building blocks of Vision-Language-Action (VLA) systems: integrating LLMs with ROS 2 for natural language understanding (Chapter 1), adding voice input with OpenAI Whisper (Chapter 2), and implementing cognitive planning for task decomposition (Chapter 3). Now it's time to integrate everything into a complete autonomous humanoid robot system.

**The Challenge**: Build a robot that can receive voice commands, generate execution plans, navigate to targets, detect objects using computer vision, and manipulate those objects—all coordinated through a single unified system. When you say "Pick up the red cup from the table," the robot should:

1. **Listen**: Transcribe your voice command using Whisper
2. **Plan**: Use an LLM to decompose the command into actionable steps
3. **Navigate**: Move to the table using Nav2 navigation
4. **Detect**: Locate the red cup using computer vision
5. **Manipulate**: Grasp the cup using MoveIt2 arm control

This isn't just a collection of separate demos—it's a cohesive system where components communicate through ROS 2 topics and actions, handle failures gracefully, and provide real-time feedback on execution progress.

**Why This Matters**: Real-world robotics applications require this level of integration. A warehouse robot needs to understand verbal instructions from workers, plan paths around obstacles, identify specific items with vision, and manipulate objects safely. An assistive home robot must interpret natural language requests, navigate cluttered environments, recognize household objects, and perform delicate manipulation tasks. The skills you develop in this capstone transfer directly to these practical applications.

**System Specifications**:
- **Input**: Natural language voice commands (English, clear speech)
- **Processing**: GPT-4 or compatible LLM for planning, Whisper for speech recognition
- **Navigation**: ROS 2 Nav2 stack with dynamic obstacle avoidance
- **Perception**: YOLO or OpenCV for object detection and localization
- **Manipulation**: MoveIt2 for trajectory planning and grasp execution
- **Feedback**: Real-time status updates via ROS 2 topics and terminal output

**Expected Outcomes**: By completing this project, you'll have a functioning autonomous humanoid system capable of end-to-end task execution from voice input to physical manipulation, with robust error handling and state management.

## System Architecture

The autonomous humanoid system consists of six integrated components coordinated by a central orchestrator:

### Component Breakdown

**1. Main Orchestrator** (`main_orchestrator.py`)
- **Role**: Central coordinator managing system state machine
- **Responsibilities**:
  - State transitions (IDLE → LISTENING → PLANNING → NAVIGATING → DETECTING → MANIPULATING → COMPLETED)
  - Component initialization and lifecycle management
  - Error propagation and recovery coordination
  - Status publishing for monitoring
- **ROS 2 Interface**:
  - Subscribes: `/voice_command` (String)
  - Publishes: `/system_status` (String), `/cmd_vel` (Twist)

**2. Voice Interface** (`voice_interface.py`)
- **Role**: Convert speech to text using OpenAI Whisper
- **Responsibilities**:
  - Audio capture from microphone
  - Real-time transcription with confidence scoring
  - Wake word detection (optional)
  - Noise filtering and voice activity detection
- **ROS 2 Interface**:
  - Publishes: `/voice_command` (String), `/transcription_confidence` (Float32)

**3. LLM Planning Module** (`llm_planning.py`)
- **Role**: Decompose commands into executable action sequences
- **Responsibilities**:
  - Few-shot prompting for task decomposition
  - JSON validation of generated plans
  - Parameter extraction (object types, locations, constraints)
  - Confidence estimation for plan quality
- **ROS 2 Interface**:
  - Service: `/plan_task` (request: String, response: ActionSequence)

**4. Navigation Controller** (`navigation_controller.py`)
- **Role**: Move robot to target locations using Nav2
- **Responsibilities**:
  - Goal pose calculation from semantic locations ("table" → coordinates)
  - Nav2 action client for `NavigateToPose`
  - Obstacle avoidance and re-planning on path blockage
  - Progress monitoring and timeout handling
- **ROS 2 Interface**:
  - Action Client: `NavigateToPose` (nav2_msgs)
  - Publishes: `/navigation_feedback` (PoseStamped)

**5. Vision Module** (`vision_module.py`)
- **Role**: Detect and localize objects in 3D space
- **Responsibilities**:
  - YOLO or OpenCV-based object detection
  - Color and shape filtering
  - Depth estimation from stereo or RGBD camera
  - Object pose publishing in robot base frame
- **ROS 2 Interface**:
  - Subscribes: `/camera/rgb/image_raw` (Image), `/camera/depth/image_raw` (Image)
  - Publishes: `/detected_objects` (DetectionArray)

**6. Manipulation Controller** (`manipulation_controller.py`)
- **Role**: Execute grasping and placement tasks
- **Responsibilities**:
  - MoveIt2 integration for trajectory planning
  - Grasp pose calculation from object pose
  - Gripper control (open/close with force sensing)
  - Collision avoidance and joint limit checking
- **ROS 2 Interface**:
  - Action Client: `MoveGroup` (moveit_msgs)
  - Publishes: `/gripper_state` (JointState)

### Data Flow Architecture

```
Voice Command → Whisper → Transcription → LLM Planner → Action Sequence
                                                            ↓
                                                  Navigation (Nav2)
                                                            ↓
                                              Vision Detection (YOLO)
                                                            ↓
                                              Manipulation (MoveIt2)
                                                            ↓
                                                     Task Complete
```

**Communication Pattern**: All components publish status updates to dedicated topics, allowing the orchestrator to monitor progress and handle failures. The system uses ROS 2 actions for long-running tasks (navigation, manipulation) to support preemption and feedback.

## Implementation Phases

### Phase 1: Voice Command Reception

**Objective**: Capture voice input and publish transcribed commands

**Code Example** (from `voice_interface.py`):
```python
import whisper
import sounddevice as sd
import numpy as np
from std_msgs.msg import String

class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')
        self.model = whisper.load_model("base")  # Load Whisper model
        self.cmd_pub = self.create_publisher(String, '/voice_command', 10)

        # Audio parameters
        self.sample_rate = 16000
        self.duration = 3  # seconds per chunk

    def listen_and_transcribe(self):
        """Capture audio and transcribe to text."""
        self.get_logger().info("Listening...")

        # Record audio
        audio = sd.rec(int(self.duration * self.sample_rate),
                      samplerate=self.sample_rate, channels=1)
        sd.wait()

        # Transcribe with Whisper
        audio_np = audio.flatten().astype(np.float32)
        result = self.model.transcribe(audio_np, fp16=False)

        # Publish if confident
        if result['segments'] and len(result['text']) > 3:
            msg = String()
            msg.data = result['text'].strip()
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
```

**Integration Point**: The orchestrator subscribes to `/voice_command` and transitions from IDLE to PLANNING state upon receiving a command.

### Phase 2: Task Planning

**Objective**: Convert natural language to structured action sequence

**Code Example** (from `llm_planning.py`):
```python
from openai import OpenAI
import json

class LLMPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')
        self.client = OpenAI()
        self.system_prompt = """Convert commands to action sequences.
Actions: navigate(location), detect(object, color), grasp(object), place(location).
Output JSON: {"plan": [{"type": "navigate", "target": "table"}, ...]}"""

    def plan_task(self, command: str) -> dict:
        """Generate action plan from command."""
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.0
        )

        # Parse JSON response
        plan_text = response.choices[0].message.content
        if "```json" in plan_text:
            plan_text = plan_text.split("```json")[1].split("```")[0]

        plan = json.loads(plan_text)
        self.get_logger().info(f"Generated {len(plan['plan'])} steps")
        return plan
```

**Key Design Decision**: Low temperature (0.0) ensures deterministic planning for safety-critical applications.

### Phase 3: Navigation Execution

**Objective**: Move robot to target location using Nav2

**Code Example** (from `navigation_controller.py`):
```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Semantic location map
        self.locations = {
            'table': (2.0, 1.0, 0.0),  # (x, y, yaw)
            'kitchen': (5.0, 3.0, 1.57),
            'bin': (1.0, -2.0, 0.0)
        }

    def navigate_to(self, location_name: str):
        """Navigate to semantic location."""
        if location_name not in self.locations:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False

        x, y, yaw = self.locations[location_name]

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = np.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = np.cos(yaw / 2)

        # Send goal and wait
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return result.accepted
```

**Error Handling**: If navigation fails (blocked path, unreachable goal), the system reports back to the orchestrator which triggers re-planning.

### Phase 4: Object Detection

**Objective**: Locate target object using computer vision

**Code Example** (from `vision_module.py`):
```python
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionModule(Node):
    def __init__(self):
        super().__init__('vision_module')
        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10)

        # Simple color detection (for demo - use YOLO in production)
        self.color_ranges = {
            'red': ((0, 100, 100), (10, 255, 255)),  # HSV ranges
            'blue': ((100, 100, 100), (130, 255, 255)),
            'green': ((40, 100, 100), (80, 255, 255))
        }

    def detect_object(self, color: str, object_type: str):
        """Detect object by color and type."""
        if not hasattr(self, 'latest_image'):
            return None

        # Convert to HSV
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

        # Color mask
        lower, upper = self.color_ranges.get(color, ((0, 0, 0), (255, 255, 255)))
        mask = cv2.inRange(hsv, lower, upper)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Largest contour
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.get_logger().info(f"Found {color} object at ({cx}, {cy})")
                return (cx, cy)

        return None

    def image_callback(self, msg):
        """Store latest camera image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
```

**Production Note**: Replace color-based detection with YOLO or Mask R-CNN for robust object recognition.

### Phase 5: Manipulation

**Objective**: Grasp detected object using MoveIt2

**Code Example** (from `manipulation_controller.py`):
```python
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')
        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')

    def grasp_object(self, object_pose):
        """Execute grasp at object pose."""
        # Pre-grasp: Open gripper
        self.set_gripper(0.08)  # Open to 8cm

        # Approach: Move to pre-grasp pose (10cm above object)
        approach_pose = object_pose.copy()
        approach_pose.position.z += 0.10
        self.move_to_pose(approach_pose)

        # Grasp: Move down to object
        self.move_to_pose(object_pose)

        # Close gripper
        self.set_gripper(0.02)  # Close around object

        # Lift: Move up
        lift_pose = object_pose.copy()
        lift_pose.position.z += 0.15
        self.move_to_pose(lift_pose)

        self.get_logger().info("Grasp complete")

    def move_to_pose(self, target_pose):
        """Move end-effector to target pose."""
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.goal_constraints = self.pose_to_constraints(target_pose)

        self.moveit_client.wait_for_server()
        future = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
```

**Safety**: All movements include collision checking and joint limit validation through MoveIt2.

## Testing and Validation

### Test Scenarios

**Scenario 1: Simple Navigation**
```
Command: "Go to the kitchen"
Expected: Robot navigates to kitchen location, publishes completion status
Validation: Check final pose within 0.5m of target
```

**Scenario 2: Object Retrieval**
```
Command: "Pick up the red cup"
Expected: Navigate → Detect → Grasp sequence completes successfully
Validation: Gripper closes with object detected by force sensors
```

**Scenario 3: Complex Multi-Step**
```
Command: "Take the blue ball from the table to the bin"
Expected: Navigate to table → Detect blue ball → Grasp → Navigate to bin → Place
Validation: Object pose changes from table to bin location
```

**Scenario 4: Error Recovery**
```
Setup: Block robot's path to target
Expected: Navigation fails → Re-planning generates alternative route
Validation: System completes task via alternate path
```

### Performance Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Voice Recognition Accuracy | >90% | Compare transcription to ground truth |
| Planning Success Rate | >95% | Valid JSON with executable actions |
| Navigation Success Rate | >85% | Reaches goal within timeout |
| Detection Accuracy | >80% | Correct object identification |
| Grasp Success Rate | >75% | Object securely held after grasp |
| End-to-End Task Completion | >70% | Full command executed successfully |
| Average Task Duration | Less than 60s | Time from voice command to completion |

### Validation Checklist

- [ ] Voice commands transcribe accurately in quiet environment
- [ ] LLM generates valid action sequences for 10+ test commands
- [ ] Robot navigates to all semantic locations without collision
- [ ] Vision system detects objects of 3+ colors correctly
- [ ] Manipulation executes pre-grasp, grasp, and lift sequence
- [ ] Error recovery triggers re-planning on navigation failure
- [ ] System status updates publish at less than 1Hz throughout execution
- [ ] All ROS 2 nodes start without errors
- [ ] Gazebo simulation runs stable for 10+ minute sessions
- [ ] Code passes pylint with score >8.0

## Extensions and Advanced Features

### Extension 1: Multi-Object Task Sequences

**Enhancement**: Support commands involving multiple objects ("Put all red objects in the bin")

**Implementation**:
```python
# In llm_planning.py - modify system prompt
self.system_prompt += """
For multi-object tasks, generate loops:
{"type": "loop", "condition": "while red objects exist",
 "actions": [detect, grasp, navigate, place]}
"""
```

**Benefits**: Enables complex cleaning, sorting, and organization tasks without manual programming.

### Extension 2: Learning from Demonstration

**Enhancement**: Record human teleoperation and extract reusable skills

**Implementation**:
- Add `/record_demo` service that logs state-action pairs
- Use imitation learning to fine-tune LLM on successful demonstrations
- Store learned skills in prompt library for few-shot examples

**Benefits**: Reduces prompt engineering effort, adapts to specific environments and object types.

### Extension 3: Multi-Modal Perception

**Enhancement**: Combine vision with tactile and force sensing

**Implementation**:
```python
# In vision_module.py
def detect_with_tactile(self, object_type):
    # Visual detection for initial localization
    visual_pose = self.detect_object(object_type)

    # Tactile exploration for precise grasp point
    tactile_data = self.touch_sensor.read()
    refined_pose = self.refine_with_touch(visual_pose, tactile_data)
    return refined_pose
```

**Benefits**: Improves grasp success rate for challenging objects (transparent, reflective, deformable).

### Extension 4: Real-Time Re-Planning

**Enhancement**: Continuous monitoring with dynamic plan adjustment

**Implementation**: Instead of open-loop execution, use closed-loop monitoring where the vision system continuously tracks object state and LLM re-plans if objects move or disappear.

**Benefits**: Robustness in dynamic environments with humans or other robots.

### Extension 5: Cloud-Based Processing

**Enhancement**: Offload heavy computation (LLM, YOLO) to cloud servers

**Implementation**:
- Deploy ROS 2 nodes on edge device (robot) and cloud server
- Use ROS 2 bridge (e.g., Zenoh) for cross-network communication
- Implement latency compensation for network delays

**Benefits**: Enables smaller onboard computers, supports more complex models.

## Integration with ROS 2

This capstone project builds on ROS 2 concepts from previous modules:

### Module 1: ROS 2 Fundamentals

**Topics and Publishers** (from [Module 1, Chapter 2](../module-01-robotic-nervous-system/02-nodes-and-topics.md)):
- All status updates use the publisher-subscriber pattern
- `/voice_command`, `/system_status`, `/detected_objects` are custom topics

**Services and Actions** (from [Module 1, Chapter 3](../module-01-robotic-nervous-system/03-services-and-actions.md)):
- Navigation uses `NavigateToPose` action for long-running tasks with feedback
- Manipulation uses `MoveGroup` action for trajectory execution
- Planning could use a service for synchronous plan requests

### Module 2: Robot Perception and Simulation

**Simulation Environment** (from [Module 2](../module-02-robot-perception/02-getting-started-with-gazebo.md)):
- Test the complete system in Gazebo before deploying to hardware
- Simulate camera sensors (from [Module 2, Sensor Simulation](../module-02-robot-perception/04-sensor-simulation-gazebo.md)) for vision module development
- Use Gazebo physics (from [Module 2, Physics Simulation](../module-02-robot-perception/03-physics-simulation.md)) for realistic manipulation testing

**World Setup**:
```xml
<!-- In world file: add table and objects -->
<model name="table">
  <pose>2.0 1.0 0.0 0 0 0</pose>
  <!-- Table mesh and collision -->
</model>
<model name="red_cup">
  <pose>2.0 1.0 0.8 0 0 0</pose>
  <!-- Cup model with red texture -->
</model>
```

### Module 3: Isaac and Navigation

**Nav2 Integration** (from [Module 3, Nav2 Basics](../module-03-isaac-navigation/06-nav2-basics.md)):
- The navigation controller uses the full Nav2 stack
- Obstacle avoidance and path planning integrated with humanoid navigation (from [Module 3, Humanoid Navigation](../module-03-isaac-navigation/07-humanoid-navigation.md))
- Perception integration for dynamic obstacle detection (from [Module 3, Isaac Perception](../module-03-isaac-navigation/05-isaac-perception.md))

**Configuration**:
- Ensure Nav2 parameters are tuned for your robot's kinematics
- Configure inflation radius for safe manipulation clearance
- Set goal tolerance appropriate for grasp accuracy requirements

## Chapter Summary

**Project Completion Checklist**:

✅ **System Components**:
- [ ] Main orchestrator coordinates all modules
- [ ] Voice interface captures and transcribes commands
- [ ] LLM planner decomposes tasks into action sequences
- [ ] Navigation controller moves robot to semantic locations
- [ ] Vision module detects and localizes objects
- [ ] Manipulation controller executes grasps

✅ **Integration**:
- [ ] All components communicate via ROS 2 topics/actions
- [ ] State machine transitions correctly through phases
- [ ] Error handling triggers appropriate recovery
- [ ] Status updates publish throughout execution

✅ **Testing**:
- [ ] Voice recognition tested with 10+ commands
- [ ] Navigation tested to all semantic locations
- [ ] Vision tested with multiple object colors
- [ ] Manipulation tested with grasp-lift-place sequence
- [ ] End-to-end tested with complete voice-to-manipulation pipeline

✅ **Documentation**:
- [ ] README with setup instructions
- [ ] Requirements.txt with all dependencies
- [ ] Configuration files (robot_params.yaml, llm_prompts.yaml)
- [ ] Code comments explaining key design decisions

**Key Learning Outcomes**:

1. **System Integration**: Coordinating multiple independent modules into a cohesive system using ROS 2's communication patterns

2. **State Management**: Implementing robust state machines that handle transitions, errors, and recovery gracefully

3. **LLM Integration**: Combining language models with robotic control systems while ensuring safety and reliability

4. **Multi-Modal Perception**: Fusing voice, vision, and proprioceptive data for comprehensive environmental understanding

5. **Error Handling**: Building resilient systems that detect failures and adapt plans dynamically

6. **Real-World Application**: Translating academic concepts into practical robotics implementations

**Common Pitfalls and Solutions**:

❌ **Pitfall**: Components work individually but fail when integrated
✅ **Solution**: Test integration incrementally—add one component at a time, verify communication, then add next

❌ **Pitfall**: State machine gets stuck in intermediate states
✅ **Solution**: Add timeout mechanisms and watchdog timers to detect hung states

❌ **Pitfall**: LLM generates invalid or unsafe actions
✅ **Solution**: Implement strict validation layer that checks action types, parameters, and feasibility before execution

❌ **Pitfall**: Network latency causes coordination issues
✅ **Solution**: Use ROS 2's QoS settings (reliable, transient local) for critical messages

**Next Steps**:

After completing this capstone, you're equipped to:
- Design and implement custom VLA systems for specific applications
- Integrate commercial LLMs (GPT-4, Claude) or local models (Llama, Mistral) with robotic platforms
- Extend the system with additional modalities (haptics, audio localization)
- Deploy to real hardware platforms (TurtleBot, PR2, Fetch, or custom robots)
- Contribute to open-source VLA projects like SayCan, Inner Monologue, or RT-2

**Congratulations!** You've built a complete autonomous humanoid system from the ground up. You've mastered the integration of natural language understanding, computer vision, navigation, and manipulation—the core pillars of modern intelligent robotics. This capstone represents the convergence of symbolic AI (language models) and embodied AI (physical robots), positioning you at the forefront of robotics research and development.

## Additional Resources

- [SayCan: Grounding Language Models in Robotic Affordances](https://say-can.github.io/)
- [RT-2: Vision-Language-Action Models](https://robotics-transformer2.github.io/)
- [Inner Monologue: Embodied Reasoning through Planning](https://arxiv.org/abs/2207.05608)
- [ROS 2 Nav2 Documentation](https://docs.nav2.org/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [YOLO Object Detection](https://github.com/ultralytics/ultralytics)

---

**Congratulations on completing Module 4: Vision-Language-Action (VLA)!** You're now ready to build the next generation of intelligent, language-enabled robots.
