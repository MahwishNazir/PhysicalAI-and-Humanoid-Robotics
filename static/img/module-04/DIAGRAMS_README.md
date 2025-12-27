# Module 4 Diagrams - VLA & Multimodal AI

This directory contains diagram placeholders for Module 4: Vision-Language-Action Models & Multimodal AI for Humanoid Robotics.

## Diagram List

### 1. VLA Architecture Diagram (`vla-architecture-diagram.png`)

**Purpose**: Illustrate the complete Vision-Language-Action model architecture showing the three main components and data flow.

**Content**:
- **Perception Module**: Camera inputs, depth sensors, vision encoders (ResNet, ViT)
- **Language Module**: Speech input (Whisper), text commands, LLM (GPT-4/Claude), language understanding
- **Action Module**: Robot primitives (navigate, grasp, place), action decoders, ROS 2 action servers
- **Data Flow**: Arrows showing perception → language grounding → task planning → action execution
- **Feedback Loop**: Action results feeding back to perception and language modules

**Style**: Technical diagram with labeled components, color-coded modules (blue=perception, green=language, orange=action)

---

### 2. Whisper Pipeline Diagram (`whisper-pipeline.png`)

**Purpose**: Show the complete speech-to-text pipeline using OpenAI Whisper integrated with ROS 2.

**Content**:
- **Input**: Microphone/audio source
- **Preprocessing**: Audio resampling (16kHz), format conversion (WAV), noise reduction
- **Whisper Model**: Encoder-decoder transformer architecture (simplified view)
- **Output**: Transcribed text with confidence scores
- **ROS 2 Integration**: Audio topic subscriber → Whisper node → Text topic publisher
- **Example**: "pick up the red cube" → structured command

**Style**: Flow diagram with processing steps, ROS 2 topic names

---

### 3. LLM Task Planning Diagram (`llm-task-planning.png`)

**Purpose**: Illustrate how LLMs convert natural language commands into structured robot task plans.

**Content**:
- **Input**: Natural language command ("organize the tools on the workbench")
- **Prompt Engineering**: System prompt + robot state + available actions + user command
- **LLM Processing**: GPT-4/Claude/Llama reasoning (shown as thought bubbles)
- **Structured Output**: JSON action sequence with steps:
  1. Navigate to workbench
  2. Detect tools (visual grounding)
  3. Pick tool 1
  4. Place in organizer slot 1
  5. Repeat for remaining tools
- **Action Translator**: Convert JSON to ROS 2 action goals

**Style**: Flow diagram with example prompt and response, JSON formatting highlighted

---

### 4. Vision-Language Grounding Diagram (`vision-language-grounding.png`)

**Purpose**: Show how CLIP/BLIP models ground natural language queries in visual perception.

**Content**:
- **CLIP Architecture**:
  - Image Encoder (Vision Transformer)
  - Text Encoder (Transformer)
  - Joint Embedding Space (cosine similarity)
- **BLIP Architecture**:
  - Image Encoder
  - Text Encoder/Decoder
  - Image-Text Matching (ITM)
  - Visual Question Answering (VQA)
- **Example Query**: "the blue wrench on the table"
- **Visual Output**: Image with bounding box highlighting detected wrench
- **3D Grounding**: 2D coordinates → depth map → 3D pose in robot frame

**Style**: Architecture diagram with example inputs/outputs, showing embedding space

---

### 5. VLA Pipeline Flow Diagram (`vla-pipeline-flow.png`)

**Purpose**: Show the complete end-to-end VLA system flow from speech input to robot action execution.

**Content**:
- **Stage 1: Speech Input**: User says "bring me the coffee mug from the kitchen table"
- **Stage 2: Transcription**: Whisper converts to text
- **Stage 3: Task Planning**: LLM generates plan:
  1. Navigate to kitchen
  2. Locate table
  3. Find coffee mug (vision-language grounding)
  4. Grasp mug
  5. Navigate to user
  6. Release mug
- **Stage 4: Visual Grounding**: CLIP/BLIP identifies "coffee mug" in scene
- **Stage 5: Action Execution**: ROS 2 action sequence (nav2, MoveIt)
- **Stage 6: Monitoring**: Continuous perception, error detection, LLM re-planning if needed
- **Feedback Loops**: Errors trigger re-planning, perception updates during execution

**Style**: Sequential flow with branching for error recovery, timeline showing latency

---

### 6. Multimodal Fusion Architecture Diagram (`multimodal-fusion-architecture.png`)

**Purpose**: Illustrate how multiple sensor modalities are fused for robust perception.

**Content**:
- **Input Modalities**:
  - RGB Camera (visual features)
  - Depth Camera (3D structure)
  - IMU (orientation, acceleration)
  - Tactile Sensors (contact, force)
  - Audio (ambient sound, speech)
- **Fusion Strategies**:
  - Early Fusion: Concatenate raw inputs
  - Late Fusion: Fuse high-level features
  - Cross-Modal Attention: Learn modality importance weights
- **Output**: Unified perception representation
- **Example**: RGB-D fusion for object segmentation in cluttered scene
- **Robustness**: Show modality dropout (what happens when camera fails)

**Style**: Architecture diagram with multiple input streams, attention weights visualization

---

### 7. Capstone System Diagram (`capstone-system-diagram.png`)

**Purpose**: Show complete autonomous humanoid system architecture for household assistant scenario.

**Content**:
- **System Layers**:
  - **Perception Layer**: Cameras, depth, IMU, microphone
  - **Processing Layer**: Whisper, CLIP/BLIP, state estimation
  - **Reasoning Layer**: LLM task planner, behavior trees
  - **Control Layer**: Navigation (nav2), manipulation (MoveIt), speech synthesis
  - **Hardware Layer**: Humanoid robot (joints, actuators, sensors)
- **Data Flows**: Sensor data → perception → reasoning → control → hardware
- **External Services**: LLM API (GPT-4/Claude), cloud services (optional)
- **State Management**: World state, task queue, robot configuration
- **Example Tasks**: "clean the table", "bring me water", "organize room"
- **Safety Layer**: Collision detection, emergency stop, failure recovery

**Style**: Comprehensive system diagram with all components, ROS 2 nodes labeled

---

## Diagram Generation Instructions

### Tools

Diagrams can be created using:
- **Draw.io / diagrams.net**: Free, web-based, export to PNG
- **Lucidchart**: Professional diagramming tool
- **Microsoft Visio**: If available
- **Python + Graphviz**: For programmatic generation
- **Mermaid**: For simple flow diagrams (can embed in docs)

### Style Guidelines

- **Color Scheme**:
  - Perception/Vision: Blue tones (#3498db, #2980b9)
  - Language/LLM: Green tones (#2ecc71, #27ae60)
  - Action/Control: Orange tones (#e67e22, #d35400)
  - Data Flow: Gray arrows (#95a5a6)
  - Error/Warning: Red tones (#e74c3c)

- **Fonts**:
  - Titles: Sans-serif, bold, 16-18pt
  - Labels: Sans-serif, regular, 12-14pt
  - Code/Commands: Monospace, 10-12pt

- **Layout**:
  - Left-to-right or top-to-bottom data flow
  - Clear component boundaries (boxes with rounded corners)
  - Adequate whitespace between components
  - Legend if using multiple symbols

### Export Settings

- **Format**: PNG with transparent background
- **Resolution**: 1920x1080 or higher (for clarity in browser)
- **DPI**: 150-300 for crisp rendering
- **Compression**: Moderate (balance file size and quality)

### Placeholder Images

Currently, all diagrams use `placeholder.png` - a simple gray box with "Module 4 Diagram Placeholder" text. Replace with actual diagrams as they are created.

## Integration with Documentation

Diagrams are referenced in chapter Markdown files using standard Markdown image syntax:

```markdown
![VLA Architecture](../../static/img/module-04/vla-architecture-diagram.png)
```

Ensure all diagram files follow the naming convention specified in this README for consistency.

## Diagram Status

| Diagram | Status | Notes |
|---------|--------|-------|
| vla-architecture-diagram.png | Placeholder | Needs creation |
| whisper-pipeline.png | Placeholder | Needs creation |
| llm-task-planning.png | Placeholder | Needs creation |
| vision-language-grounding.png | Placeholder | Needs creation |
| vla-pipeline-flow.png | Placeholder | Needs creation |
| multimodal-fusion-architecture.png | Placeholder | Needs creation |
| capstone-system-diagram.png | Placeholder | Needs creation |

## TODO

- [ ] Create all 7 diagrams using preferred tool
- [ ] Export at appropriate resolution
- [ ] Replace placeholder.png references in chapters
- [ ] Verify diagrams render correctly in Docusaurus build
- [ ] Add diagram captions in chapter content

## Updated: 2025-12-26

For questions about diagram requirements, refer to the specification (spec.md) or implementation plan (plan.md).
