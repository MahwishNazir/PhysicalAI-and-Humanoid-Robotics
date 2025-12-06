# Feature Specification: Chapter 2 - Nodes & Topics (Comprehensive Educational Content)

**Feature Branch**: `004-ros2-nodes-topics-chapter`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "Act as an Author who is writing and explaining the chapter 2: 'Nodes & Topics'. Must cover all its content, if need to make sub chapters or part of chapter so make it, if not required then ok. Explain each topic with examples and write code where required"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learning Node Architecture and Communication (Priority: P1)

A learner who completed Chapter 1 needs to understand how ROS 2 nodes work in depth, including their lifecycle, how they communicate through topics, and how to create their own nodes with publishers and subscribers.

**Why this priority**: This is the core learning journey. Understanding nodes and topics is fundamental to all ROS 2 development. Without this knowledge, learners cannot build functional robot systems.

**Independent Test**: Can be fully tested by having a learner who completed Chapter 1 read through the chapter, execute all code examples to create working publisher/subscriber nodes, and successfully modify examples to create their own simple communication system.

**Acceptance Scenarios**:

1. **Given** a learner with Chapter 1 knowledge, **When** they read about node architecture, **Then** they understand what a node is, its lifecycle, and how nodes interact in a ROS 2 system
2. **Given** a learner following publisher code examples, **When** they create and run a publisher node, **Then** the node successfully publishes messages to a topic at the specified rate
3. **Given** a learner following subscriber code examples, **When** they create and run a subscriber node, **Then** the node receives and processes messages from a topic
4. **Given** a learner understanding both concepts, **When** they run publisher and subscriber nodes simultaneously, **Then** they observe real-time communication between nodes
5. **Given** a learner with basic knowledge, **When** they modify code examples to change message content or topics, **Then** they successfully customize node behavior

---

### User Story 2 - Understanding Message Types and Custom Messages (Priority: P1)

A learner needs to understand ROS 2 message types, how to use standard message types, and how to create custom message types for their specific robot application needs.

**Why this priority**: Message types are essential for meaningful communication. Learners must know how to select appropriate standard messages and create custom ones when needed.

**Independent Test**: Can be tested by having a learner identify appropriate standard message types for common scenarios, create a custom message definition, build it, and use it in publisher/subscriber nodes.

**Acceptance Scenarios**:

1. **Given** a learner understanding topics, **When** they explore standard ROS 2 message types (std_msgs, geometry_msgs, sensor_msgs), **Then** they can select appropriate message types for common use cases
2. **Given** a learner with unique data requirements, **When** they follow custom message creation instructions, **Then** they successfully define, build, and use a custom message type in their nodes
3. **Given** a learner working with messages, **When** they examine message structure, **Then** they understand message fields, data types, and how to access them in code
4. **Given** a learner debugging communication, **When** they use `ros2 interface show` command, **Then** they can inspect message type definitions to understand structure

---

### User Story 3 - Mastering Quality of Service (QoS) Policies (Priority: P2)

A learner building reliable robot systems needs to understand QoS policies and how to configure reliability, durability, and history settings to ensure proper communication under various network and timing conditions.

**Why this priority**: QoS is critical for reliable systems but builds on basic pub-sub understanding. It's essential for real-world applications but can be learned after basic communication works.

**Independent Test**: Can be tested by having a learner configure different QoS profiles (reliable, best-effort), test behavior under simulated packet loss, and explain when to use each profile.

**Acceptance Scenarios**:

1. **Given** a learner with working pub-sub nodes, **When** they learn about QoS policies, **Then** they understand reliability, durability, history, and deadline policies and their use cases
2. **Given** a learner configuring QoS, **When** they set reliability to RELIABLE vs BEST_EFFORT, **Then** they observe different behavior and understand trade-offs
3. **Given** a learner building safety-critical systems, **When** they apply appropriate QoS profiles, **Then** they ensure critical messages are not lost
4. **Given** a learner debugging communication issues, **When** they encounter QoS incompatibility, **Then** they can diagnose and fix the mismatch

---

### User Story 4 - Using ROS 2 Tools for Topic Debugging (Priority: P2)

A learner developing multi-node systems needs to use ROS 2 command-line tools and GUI tools to list topics, inspect messages, visualize communication graphs, and debug communication issues.

**Why this priority**: Debugging tools are essential for development but secondary to understanding core concepts. These are supporting skills that enhance productivity.

**Independent Test**: Can be tested by giving a learner a multi-node system with communication issues and having them use `ros2 topic`, `ros2 node`, and `rqt_graph` to diagnose and explain the problem.

**Acceptance Scenarios**:

1. **Given** a running ROS 2 system, **When** a learner uses `ros2 topic list`, **Then** they see all active topics and understand topic naming conventions
2. **Given** an active topic, **When** a learner uses `ros2 topic echo`, **Then** they can inspect message content in real-time
3. **Given** a multi-node system, **When** a learner uses `rqt_graph`, **Then** they visualize node-topic relationships and identify communication patterns
4. **Given** communication problems, **When** a learner uses `ros2 topic info` and `ros2 topic hz`, **Then** they can diagnose message rate and connection issues

---

### User Story 5 - Designing Multi-Node System Architecture (Priority: P3)

An advanced learner or developer needs to understand design patterns for organizing multiple nodes, managing topics effectively, avoiding common pitfalls, and structuring scalable robot systems.

**Why this priority**: System design patterns are important but require solid foundation in basic node and topic mechanics. This is advanced material suitable after mastering fundamentals.

**Independent Test**: Can be tested by having a learner design a multi-node system for a specific robot application (e.g., autonomous navigation), justify topic structure and node responsibilities, and implement a working prototype.

**Acceptance Scenarios**:

1. **Given** a robot application requirement, **When** a learner designs the node architecture, **Then** they appropriately separate concerns into logical nodes (sensing, processing, control)
2. **Given** multiple nodes communicating, **When** a learner designs topic names, **Then** they follow naming conventions and organize topics logically
3. **Given** a complex system, **When** a learner implements it, **Then** they avoid common anti-patterns (circular dependencies, topic storms, single point of failure)
4. **Given** a learner understanding patterns, **When** they refactor code, **Then** they can improve system modularity and maintainability

---

### User Story 6 - Quick Reference for Experienced Developers (Priority: P3)

An experienced developer already familiar with node and topic concepts needs quick access to syntax, command reference, QoS parameters, and best practices without reading the full chapter.

**Why this priority**: Supporting quick reference improves chapter utility for diverse audiences but is not the primary learning objective.

**Independent Test**: Can be tested by asking an experienced developer to find specific information (e.g., "How to set QoS to transient local?") within 30 seconds using the summary section.

**Acceptance Scenarios**:

1. **Given** a developer needing quick syntax, **When** they scan the summary section, **Then** they find publisher/subscriber code templates with key parameters
2. **Given** a developer troubleshooting, **When** they consult the tools reference, **Then** they find all relevant `ros2 topic` and `ros2 node` commands with examples
3. **Given** a developer configuring QoS, **When** they check the QoS reference, **Then** they find all policy options with recommended use cases

---

### Edge Cases

- What happens when a learner doesn't have ROS 2 Humble installed?
  - Specify Humble as the target distribution but note commands work on Foxy, Galactic, Iron
  - Provide installation validation steps before attempting examples
  - Include "Prerequisites" section requiring Chapter 1 completion and ROS 2 installation

- How does the chapter handle learners who prefer Python vs C++?
  - Provide all core examples in both Python and C++
  - Default to Python for primary examples (more beginner-friendly)
  - Use clear language model-agnostic concepts before diving into code

- What if learners cannot run multi-terminal commands (IDE limitations)?
  - Explain launch files as an alternative (preview of Chapter 6)
  - Provide instructions for both terminal-based and IDE-based workflows
  - Include tmux/screen recommendations for Linux users

- How to handle learners who want to go deeper into DDS internals?
  - Provide "Going Deeper" or "Advanced Topics" callout boxes
  - Link to external resources (ROS 2 docs, DDS specifications) for deep dives
  - Keep main content focused on practical usage, not theory

- What if code examples fail due to environment issues?
  - Include troubleshooting section for common errors
  - Provide validation commands before running examples (`ros2 doctor`, workspace sourcing)
  - Include expected output for each example so learners can verify success

- How to keep content relevant as ROS 2 evolves?
  - Use ROS 2 Humble (LTS until 2027) for maximum stability
  - Note features specific to newer distributions where applicable
  - Avoid hard-coded version-specific paths or commands

## Requirements *(mandatory)*

### Functional Requirements

#### Content Structure

- **FR-001**: Chapter MUST be organized into two distinct sections: "Full Lesson" and "Summary"
- **FR-002**: Full Lesson MUST use subsections to organize major topics (Node Architecture, Topics & Communication, Message Types, QoS Policies, Debugging Tools, System Design)
- **FR-003**: Summary section MUST provide concise bullet-point recap of all major concepts, commands, and code patterns
- **FR-004**: Content MUST progress logically from simple concepts (what is a node?) to advanced topics (multi-node system design)
- **FR-005**: Each major section MUST have clear learning objectives stated at the beginning

#### Node Architecture Content

- **FR-006**: Chapter MUST explain what a ROS 2 node is and its role in the system
- **FR-007**: Chapter MUST explain node lifecycle states (unconfigured, inactive, active, finalized)
- **FR-008**: Chapter MUST explain node initialization, execution, and shutdown processes
- **FR-009**: Chapter MUST explain rclpy (Python) and rclcpp (C++) node APIs
- **FR-010**: Chapter MUST explain node naming, namespaces, and remapping

#### Topics and Communication Content

- **FR-011**: Chapter MUST explain the publish-subscribe pattern in depth with diagrams
- **FR-012**: Chapter MUST explain what topics are, their purpose, and naming conventions
- **FR-013**: Chapter MUST explain the difference between synchronous and asynchronous communication
- **FR-014**: Chapter MUST explain many-to-many communication (multiple publishers/subscribers)
- **FR-015**: Chapter MUST explain topic discovery mechanisms and how nodes find each other

#### Message Types Content

- **FR-016**: Chapter MUST explain ROS 2 message types and their structure
- **FR-017**: Chapter MUST cover common standard message packages (std_msgs, geometry_msgs, sensor_msgs)
- **FR-018**: Chapter MUST explain message field types (primitives, arrays, nested messages)
- **FR-019**: Chapter MUST teach how to create custom message definitions (.msg files)
- **FR-020**: Chapter MUST explain how to build and use custom messages in packages
- **FR-021**: Chapter MUST explain message headers, timestamps, and frame IDs

#### Code Examples - Publishers

- **FR-022**: Chapter MUST include complete Python publisher node example with initialization, timer, and publishing logic
- **FR-023**: Chapter MUST include complete C++ publisher node example with equivalent functionality
- **FR-024**: Publisher examples MUST demonstrate setting publish rate/frequency
- **FR-025**: Publisher examples MUST demonstrate using standard message types (e.g., String, Int32, Twist)
- **FR-026**: Chapter MUST include example of publisher using custom message type
- **FR-027**: All publisher code MUST include inline comments explaining key lines
- **FR-028**: Publisher examples MUST include package setup (package.xml, CMakeLists.txt/setup.py)

#### Code Examples - Subscribers

- **FR-029**: Chapter MUST include complete Python subscriber node example with callback function
- **FR-030**: Chapter MUST include complete C++ subscriber node example with equivalent functionality
- **FR-031**: Subscriber examples MUST demonstrate callback functions and message handling
- **FR-032**: Subscriber examples MUST demonstrate accessing message fields
- **FR-033**: Chapter MUST include example of subscriber using custom message type
- **FR-034**: All subscriber code MUST include inline comments explaining key lines

#### Code Examples - Integration

- **FR-035**: Chapter MUST include example combining publisher and subscriber in a single node
- **FR-036**: Chapter MUST provide instructions for running multiple nodes in separate terminals
- **FR-037**: All code examples MUST be tested and functional in ROS 2 Humble
- **FR-038**: Code examples MUST follow ROS 2 coding conventions and best practices
- **FR-039**: Code examples MUST include expected output so learners can verify correctness

#### Quality of Service (QoS) Content

- **FR-040**: Chapter MUST explain what QoS policies are and why they matter
- **FR-041**: Chapter MUST explain reliability policies (RELIABLE vs BEST_EFFORT) with use cases
- **FR-042**: Chapter MUST explain durability policies (TRANSIENT_LOCAL vs VOLATILE) with use cases
- **FR-043**: Chapter MUST explain history policies (KEEP_LAST vs KEEP_ALL) with use cases
- **FR-044**: Chapter MUST explain deadline, lifespan, and liveliness policies
- **FR-045**: Chapter MUST provide code examples showing how to set QoS profiles in Python and C++
- **FR-046**: Chapter MUST explain QoS compatibility and what happens when profiles mismatch
- **FR-047**: Chapter MUST provide recommendations for common scenarios (sensor data, commands, critical messages)

#### Debugging and Tools Content

- **FR-048**: Chapter MUST explain how to use `ros2 node list` to see active nodes
- **FR-049**: Chapter MUST explain how to use `ros2 node info` to inspect node details
- **FR-050**: Chapter MUST explain how to use `ros2 topic list` to see active topics
- **FR-051**: Chapter MUST explain how to use `ros2 topic echo` to inspect message content
- **FR-052**: Chapter MUST explain how to use `ros2 topic info` to see topic details and connections
- **FR-053**: Chapter MUST explain how to use `ros2 topic hz` to measure message publication rate
- **FR-054**: Chapter MUST explain how to use `ros2 topic pub` to manually publish messages
- **FR-055**: Chapter MUST explain how to use `rqt_graph` to visualize node-topic graph
- **FR-056**: Chapter MUST include screenshots or diagrams showing tool outputs

#### System Design Content

- **FR-057**: Chapter MUST explain principles for deciding when to split functionality into separate nodes
- **FR-058**: Chapter MUST explain topic naming conventions and namespace organization
- **FR-059**: Chapter MUST explain common multi-node design patterns (pipeline, star, hierarchical)
- **FR-060**: Chapter MUST explain common anti-patterns and pitfalls to avoid
- **FR-061**: Chapter MUST provide example of a complete multi-node system design with justification

#### Learning Aids

- **FR-062**: Chapter MUST include hands-on exercises for learners to practice creating nodes
- **FR-063**: Chapter MUST include hands-on exercises for learners to practice debugging with tools
- **FR-064**: Complex concepts MUST be accompanied by diagrams (node lifecycle, pub-sub pattern, QoS flow)
- **FR-065**: Technical terms MUST be defined on first use or linked to glossary
- **FR-066**: Chapter MUST include "Prerequisites" section requiring Chapter 1 completion
- **FR-067**: Chapter MUST include "Learning Objectives" section at the beginning
- **FR-068**: Chapter MUST include "Key Takeaways" section at the end of Full Lesson
- **FR-069**: Chapter MUST include troubleshooting section addressing common errors

#### Accessibility and Format

- **FR-070**: All code examples MUST use proper markdown syntax highlighting (```python, ```cpp)
- **FR-071**: All commands MUST be formatted in code blocks for easy copying
- **FR-072**: Chapter MUST include estimated reading/completion time in front matter
- **FR-073**: Chapter MUST include tags and keywords for searchability
- **FR-074**: Diagrams MUST have alt text for accessibility

### Key Entities

- **Full Lesson Section**: Comprehensive, in-depth coverage of nodes and topics with detailed explanations, examples, code, diagrams
  - Contains: Node architecture, topics & communication, message types, QoS, debugging tools, system design
  - Relationship: Parent section containing subsections organized by major topic

- **Summary Section**: Concise recap of Full Lesson in bullet-point format
  - Contains: Key concepts, essential commands, code templates, QoS reference, troubleshooting quick reference
  - Relationship: Condensed version of Full Lesson content for quick reference

- **Code Example**: Working, tested code snippet demonstrating a concept
  - Attributes: Language (Python/C++), concept demonstrated (publisher/subscriber/QoS), inline comments, package setup
  - Relationship: Embedded within Full Lesson subsections

- **Diagram**: Visual illustration of concepts
  - Attributes: Type (node lifecycle, pub-sub pattern, system architecture, tool output), format (mermaid, image), alt text
  - Relationship: Embedded within Full Lesson to support textual explanations

- **Exercise**: Hands-on activity for learners to practice
  - Attributes: Difficulty level, estimated time, required tools, learning objective, expected outcome
  - Relationship: Placed after relevant concept explanation

- **Tool Reference**: Documentation for ROS 2 CLI or GUI tools
  - Attributes: Tool name, command syntax, example usage, output interpretation
  - Relationship: Organized in debugging tools section

- **Design Pattern**: Reusable solution for organizing nodes and topics
  - Attributes: Pattern name, use case, structure, advantages/disadvantages
  - Relationship: Organized in system design section

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Learners can create a working publisher node (Python or C++) that publishes to a topic after reading the chapter
- **SC-002**: Learners can create a working subscriber node (Python or C++) that receives messages from a topic after reading the chapter
- **SC-003**: Learners can successfully run publisher and subscriber nodes simultaneously and observe communication
- **SC-004**: Learners can identify and use appropriate standard ROS 2 message types for common scenarios
- **SC-005**: Learners can create, build, and use a custom message type in their nodes
- **SC-006**: Learners can configure QoS policies and explain when to use RELIABLE vs BEST_EFFORT
- **SC-007**: Learners can use `ros2 topic` commands to list, inspect, and debug active topics
- **SC-008**: Learners can use `rqt_graph` to visualize node-topic relationships
- **SC-009**: Learners can diagnose common communication issues (no connection, QoS mismatch, wrong topic name)
- **SC-010**: Learners can design a simple multi-node system with justified node/topic structure
- **SC-011**: All code examples execute without errors in ROS 2 Humble environment
- **SC-012**: Learners can recall key concepts (node lifecycle, QoS policies, debugging commands) using only summary section
- **SC-013**: At least 90% of learners successfully complete hands-on exercises on first attempt
- **SC-014**: Experienced developers can find specific information (syntax, commands) within 1 minute using summary
- **SC-015**: Chapter reading and exercise completion time is between 45-75 minutes for learners who completed Chapter 1

## Assumptions

- Learners have completed Chapter 1 and understand basic ROS 2 concepts
- Learners have ROS 2 Humble Hawksbill installed and working (or equivalent LTS version)
- Learners have basic programming knowledge in Python or C++
- Learners can run multiple terminal windows or use terminal multiplexers
- Learners have colcon build tools installed for building custom messages
- Default language for examples: Python (more accessible for beginners)
- C++ examples are provided as supplementary learning for those interested
- Learners have completed workspace setup (Chapter 1 or setup guide)
- Visual Studio Code or similar IDE is available but not required
- Diagrams use Mermaid syntax or embedded images for maximum compatibility

## Out of Scope

- **Not Included**: Advanced lifecycle node management (lifecycle states are introduced but deep control is Chapter 4+)
- **Not Included**: ROS 2 Actions - covered in Chapter 3: Services and Actions
- **Not Included**: ROS 2 Services - covered in Chapter 3: Services and Actions
- **Not Included**: Parameters and parameter servers - covered in Chapter 4
- **Not Included**: Launch files - covered in Chapter 6: Launch Files
- **Not Included**: Advanced DDS configuration and tuning - beyond introductory scope
- **Not Included**: Real-time performance tuning - covered in advanced modules
- **Not Included**: Security and encryption - covered in advanced modules
- **Not Included**: Cross-language communication details (Python-C++ interop is automatic)
- **Not Included**: ROS 1 bridge - not relevant for ROS 2-focused book
- **Not Included**: Video/multimedia content - focus on written explanations and code examples
- **Not Included**: Interactive quizzes or assessments - exercises provide hands-on practice

## Notes

- Content must maintain beginner-friendly tone while being technically accurate
- Balance depth and accessibility - explain complex concepts clearly without oversimplifying
- Ensure consistency with Chapter 1 style, structure, and pedagogical approach
- Build on Chapter 1 knowledge - reference concepts introduced there rather than repeating
- Full Lesson should be comprehensive but focused - aim for 5000-7000 words
- Summary should fit on 2-3 pages maximum for quick reference
- Code examples should be minimal but complete - avoid overwhelming learners with boilerplate
- Emphasize hands-on learning - learners should spend more time coding than reading
- Use consistent terminology with ROS 2 official documentation
- All diagrams should be simple, clear, and directly support understanding
- Troubleshooting section should address actual common errors learners encounter
- Design patterns should be practical and immediately applicable, not theoretical
- Consider learners from diverse backgrounds - use clear examples and avoid cultural assumptions
