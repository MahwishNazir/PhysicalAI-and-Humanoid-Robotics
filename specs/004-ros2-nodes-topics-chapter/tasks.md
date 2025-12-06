# Implementation Tasks: Chapter 2 - Nodes & Topics

**Feature Branch**: `004-ros2-nodes-topics-chapter`
**Date**: 2025-12-05
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This document breaks down the implementation of Chapter 2: "Nodes & Topics" into specific, executable tasks organized by user story. Each phase represents an independently testable increment of educational content.

**Target Deliverable**: `docs/module-01-robotic-nervous-system/02-nodes-and-topics.md`

**Total Estimated Tasks**: 38 tasks
**MVP Scope**: User Story 1 (Phase 3) - Core node and topic concepts
**Parallel Opportunities**: 26 parallelizable tasks (marked with [P])

---

## Phase 1: Setup & Initialization (4 tasks)

**Goal**: Prepare chapter file structure and front matter

**Tasks**:

- [ ] T001 Create chapter front matter in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Add YAML front matter with title, tags, keywords, difficulty, estimated_time, prerequisites, learning_objectives
  - Use template from quickstart.md
  - Verify Docusaurus compatibility

- [ ] T002 [P] Create assets directory structure at docs/module-01-robotic-nervous-system/_assets/diagrams/
  - Create _assets/diagrams/ for Mermaid source files
  - Create _assets/code/python/ for complete Python examples
  - Create _assets/code/cpp/ for complete C++ examples

- [ ] T003 [P] Create screenshots directory at static/img/module-01/chapter-02/
  - Create static/img/module-01/chapter-02/ for tool output screenshots
  - Prepare placeholder structure for rqt_graph, ros2 commands

- [ ] T004 Review data-model.md and quickstart.md for content structure reference
  - Familiarize with hierarchical outline (13 subsections)
  - Review code templates and diagram specifications
  - Review quality checklist for each section

---

## Phase 2: Foundational Content (No blocking prerequisites for user stories)

**Note**: No foundational phase needed. All user stories can begin after Phase 1 setup.

---

## Phase 3: User Story 1 - Learning Node Architecture and Communication (Priority: P1)

**Goal**: Enable learners to understand node architecture, create publishers/subscribers, and observe communication

**Independent Test**: Learner reads sections 1.1-1.7, executes all code examples, creates working pub-sub nodes, modifies examples successfully

**Tasks**:

### Prerequisites & Learning Objectives

- [ ] T005 [US1] Write section 1.1: Prerequisites & Learning Objectives in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - List prerequisites (Chapter 1, ROS 2 Humble, Python/C++ basics)
  - List 5-6 learning objectives from data-model.md section 1.1
  - Target: 150 words
  - FR-066, FR-067

### Node Architecture

- [ ] T006 [P] [US1] Write section 1.2.1: What is a Node? in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Define node as independent process
  - Explain role in ROS 2 system
  - Provide camera/processor/controller examples
  - Target: 200 words
  - FR-006

- [ ] T007 [P] [US1] Create Mermaid diagram for simple 3-node system and save to docs/module-01-robotic-nervous-system/_assets/diagrams/simple-3node-system.mmd
  - Show Camera → Processor → Controller communication
  - Embed in section 1.2.1
  - Include alt text for accessibility

- [ ] T008 [P] [US1] Write section 1.2.2: Node Lifecycle States in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain managed vs non-managed nodes
  - Describe lifecycle states (Unconfigured, Inactive, Active, Finalized)
  - List state transitions
  - Note focus on concept, not implementation
  - Target: 250 words
  - FR-007

- [ ] T009 [P] [US1] Create Mermaid state diagram for node lifecycle and save to docs/module-01-robotic-nervous-system/_assets/diagrams/node-lifecycle.mmd
  - Use syntax from research.md Section 1
  - Embed in section 1.2.2
  - Include alt text

- [ ] T010 [P] [US1] Write section 1.2.3: Node Initialization & Execution in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain rclpy.init() / rclcpp::init()
  - Describe node instance creation
  - Explain spinning (event loop)
  - Describe shutdown process
  - Include basic Python node skeleton (10 lines)
  - Target: 200 words
  - FR-008

- [ ] T011 [P] [US1] Write section 1.2.4: Node Naming & Namespaces in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain node names as unique identifiers
  - Describe namespaces and hierarchy
  - Explain remapping
  - Provide examples (/robot1/camera)
  - Include ros2 node list, ros2 node info commands
  - Target: 150 words
  - FR-009, FR-010

### Topics & Communication

- [ ] T012 [P] [US1] Write section 1.3.1: Publish-Subscribe Pattern in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain decoupled communication concept
  - Use radio broadcast analogy
  - List advantages (loose coupling, scalability, flexibility)
  - Contrast with request-reply (services)
  - Target: 250 words
  - FR-011

- [ ] T013 [P] [US1] Create Mermaid flowchart for publish-subscribe pattern and save to docs/module-01-robotic-nervous-system/_assets/diagrams/pubsub-pattern.mmd
  - Use syntax from research.md Section 4
  - Show publishers → topic → subscribers
  - Embed in section 1.3.1

- [ ] T014 [P] [US1] Write section 1.3.2: Topic Naming Conventions in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain topic naming (descriptive, hierarchical)
  - Describe conventions (snake_case, leading slash)
  - Provide examples (/camera/image_raw, /cmd_vel)
  - List best practices
  - Target: 150 words
  - FR-012

- [ ] T015 [P] [US1] Write section 1.3.3: Many-to-Many Communication in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain multiple publishers to one topic
  - Explain one publisher to multiple subscribers
  - Describe many-to-many flexibility
  - Provide use case examples
  - Target: 150 words
  - FR-014

- [ ] T016 [P] [US1] Write section 1.3.4: Topic Discovery in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain DDS discovery mechanism (abstracted)
  - Describe automatic node discovery
  - Explain no central broker required
  - Include ros2 topic list, ros2 topic info commands
  - Target: 150 words
  - FR-013, FR-015

### Publishers

- [ ] T017 [US1] Write section 1.5.1: Python Publisher Example in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Complete MinimalPublisher class example (~30 lines + comments)
  - Follow code template from quickstart.md
  - Explain each component (publisher, timer, callback, main)
  - Include expected output
  - Target: 350 words
  - FR-022, FR-024, FR-025, FR-027

- [ ] T018 [P] [US1] Write section 1.5.2: C++ Publisher Example in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Complete MinimalPublisher class example (~40 lines + comments)
  - Follow C++ template from quickstart.md
  - Highlight key differences from Python (shared pointers, member initialization)
  - Include expected output
  - Target: 250 words
  - FR-023

- [ ] T019 [P] [US1] Write section 1.5.3: Package Setup in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain Python package structure (package.xml, setup.py)
  - Explain C++ package structure (package.xml, CMakeLists.txt)
  - List dependencies (rclpy/rclcpp, std_msgs)
  - Include colcon build command
  - Include ros2 run command
  - Target: 100 words
  - FR-028

### Subscribers

- [ ] T020 [US1] Write section 1.6.1: Python Subscriber Example in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Complete MinimalSubscriber class example (~25 lines + comments)
  - Follow code template from quickstart.md
  - Explain subscription creation and callback
  - Include expected output when publisher running
  - Target: 300 words
  - FR-029, FR-031, FR-032, FR-034

- [ ] T021 [P] [US1] Write section 1.6.2: C++ Subscriber Example in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Complete MinimalSubscriber class example (~35 lines + comments)
  - Highlight key differences (callback signature, std::bind)
  - Include expected output
  - Target: 200 words
  - FR-030

- [ ] T022 [P] [US1] Write section 1.6.3: Callback Functions in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain callback as function executed on message arrival
  - Describe asynchronous nature
  - List best practices (keep short, don't block)
  - Mention threading considerations
  - Target: 100 words
  - FR-031

### Integration

- [ ] T023 [US1] Write section 1.7.1: Combined Publisher-Subscriber Node in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Describe use case (data transformation pipeline)
  - Complete example (~50 lines) with both pub and sub
  - Show callback receives, transforms, publishes
  - Include multi-terminal running instructions
  - Create simple pipeline diagram (3 nodes)
  - Target: 300 words
  - FR-035, FR-036

- [ ] T024 [US1] Test all code examples from sections 1.5-1.7 in ROS 2 Humble environment
  - Execute Python publisher, subscriber, combined node
  - Execute C++ publisher, subscriber
  - Verify expected outputs match actual outputs
  - Update any discrepancies in chapter content
  - FR-037, FR-038, FR-039

---

## Phase 4: User Story 2 - Understanding Message Types and Custom Messages (Priority: P1)

**Goal**: Enable learners to work with standard message types and create custom messages

**Independent Test**: Learner identifies appropriate standard messages, creates custom message definition, builds it, uses in pub-sub nodes

**Dependencies**: User Story 1 (needs pub-sub foundation)

**Tasks**:

- [ ] T025 [P] [US2] Write section 1.4.1: ROS 2 Message Structure in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Define messages as data structures
  - Explain strongly typed communication
  - Describe field types (primitives, arrays, nested)
  - Use geometry_msgs/Twist as example
  - Include ros2 interface show command
  - Target: 150 words
  - FR-016, FR-018

- [ ] T026 [P] [US2] Write section 1.4.2: Standard Message Packages in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - List std_msgs, geometry_msgs, sensor_msgs, nav_msgs
  - Provide examples with use cases
  - Explain when to use standard vs custom
  - Target: 200 words
  - FR-017

- [ ] T027 [P] [US2] Write section 1.4.3: Custom Messages in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain when to create custom message
  - Show .msg file syntax with PersonInfo example
  - Describe building process (colcon build)
  - Explain package structure (msg/ directory)
  - Include package.xml dependencies
  - Target: 200 words
  - FR-019, FR-020

- [ ] T028 [P] [US2] Write section 1.4.4: Headers & Timestamps in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain std_msgs/Header (timestamp + frame_id)
  - Describe synchronization use
  - Mention ROS time vs system time
  - Provide sensor_msgs/Image example
  - Target: 50 words
  - FR-021

- [ ] T029 [P] [US2] Add custom message example to section 1.5.1 in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Show publisher using custom PersonInfo message
  - Include .msg file definition
  - Show import and usage
  - FR-026

- [ ] T030 [P] [US2] Add custom message example to section 1.6.1 in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Show subscriber using custom PersonInfo message
  - Show accessing custom message fields
  - FR-033

- [ ] T031 [US2] Create Exercise 2: Custom Message Type in section 1.11 in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Objective: Create and use custom message
  - Steps: Define PersonInfo.msg, build, create pub-sub using it
  - Expected outcome, difficulty (Intermediate), time (30 min)
  - FR-062

---

## Phase 5: User Story 3 - Mastering Quality of Service (QoS) Policies (Priority: P2)

**Goal**: Enable learners to configure QoS policies for reliable communication

**Independent Test**: Learner configures different QoS profiles, tests behavior, explains trade-offs, debugs QoS incompatibility

**Dependencies**: User Story 1 (needs pub-sub foundation)

**Tasks**:

- [ ] T032 [P] [US3] Write section 1.8.1: What are QoS Policies? in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Define QoS as fine-grained control
  - Explain why needed (different use cases)
  - Mention DDS foundation
  - List policy types (Reliability, Durability, History, Deadline, Lifespan, Liveliness)
  - Target: 150 words
  - FR-040

- [ ] T033 [P] [US3] Write section 1.8.2: Reliability Policies in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain RELIABLE (guaranteed delivery, retransmission)
  - Explain BEST_EFFORT (no guarantee, fire-and-forget)
  - Provide use cases (commands vs sensor data)
  - Show trade-offs (latency vs reliability)
  - Include code example setting reliability
  - Target: 200 words
  - FR-041, FR-045

- [ ] T034 [P] [US3] Write section 1.8.3: Durability & History in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain VOLATILE vs TRANSIENT_LOCAL durability
  - Explain KEEP_LAST(n) vs KEEP_ALL history
  - Provide use cases (late joiners, buffering)
  - Include code example setting durability and history
  - Target: 200 words
  - FR-042, FR-043, FR-044, FR-045

- [ ] T035 [P] [US3] Write section 1.8.4: QoS Configuration Examples in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Show Python QoS profile creation
  - Show C++ QoS profile creation
  - Demonstrate using predefined profiles
  - Complete code example with custom QoS
  - Target: 150 words
  - FR-045

- [ ] T036 [P] [US3] Write section 1.8.5: QoS Compatibility in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain compatibility rules (RELIABLE ↔ RELIABLE/BEST_EFFORT)
  - Describe incompatibility consequences (no connection)
  - Show debugging with ros2 topic info --verbose
  - Include QoS compatibility sequence diagram from research.md
  - Target: 100 words
  - FR-046, FR-047

- [ ] T037 [P] [US3] Create Mermaid sequence diagram for QoS compatibility and save to docs/module-01-robotic-nervous-system/_assets/diagrams/qos-compatibility.mmd
  - Use syntax from research.md Section 4
  - Show QoS negotiation flow
  - Embed in section 1.8.5

- [ ] T038 [P] [US3] Create Exercise 3: QoS Experimentation in section 1.11 in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Objective: Observe QoS policy effects
  - Steps: Create BEST_EFFORT pub, RELIABLE sub, observe incompatibility, fix
  - Expected outcome, difficulty (Intermediate), time (15 min)
  - FR-063

- [ ] T039 [US3] Test QoS code examples from section 1.8 in ROS 2 Humble environment
  - Test RELIABLE and BEST_EFFORT examples
  - Test QoS incompatibility scenario
  - Verify expected behaviors
  - FR-037

---

## Phase 6: User Story 4 - Using ROS 2 Tools for Topic Debugging (Priority: P2)

**Goal**: Enable learners to use ROS 2 debugging tools

**Independent Test**: Given multi-node system with issues, learner uses tools to diagnose and explain problem

**Dependencies**: User Story 1 (needs nodes/topics running)

**Tasks**:

- [ ] T040 [P] [US4] Write section 1.9.1: ros2 node Commands in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain ros2 node list command
  - Explain ros2 node info command
  - Provide example output with explanation
  - Describe use case (verify node running, check connections)
  - Target: 150 words
  - FR-048, FR-049

- [ ] T041 [P] [US4] Write section 1.9.2: ros2 topic Commands in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain ros2 topic list, echo, info, hz, pub, bw commands
  - Provide example workflow for debugging
  - Include sample command outputs
  - Target: 300 words
  - FR-050, FR-051, FR-052, FR-053, FR-054

- [ ] T042 [P] [US4] Capture screenshots of ros2 command outputs and save to static/img/module-01/chapter-02/
  - ros2 topic list output
  - ros2 topic echo output
  - ros2 topic info output
  - Embed in section 1.9.2
  - FR-056

- [ ] T043 [P] [US4] Write section 1.9.3: rqt_graph Visualization in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain graphical tool for visualizing node-topic graph
  - Describe how to interpret graph (ovals=nodes, rectangles=topics, arrows=connections)
  - Mention filters
  - Describe use case (understanding architecture at a glance)
  - Target: 150 words
  - FR-055

- [ ] T044 [P] [US4] Capture rqt_graph screenshot and save to static/img/module-01/chapter-02/rqt-graph-example.png
  - Show multi-node system with topics
  - Embed in section 1.9.3
  - Include alt text
  - FR-056

---

## Phase 7: User Story 5 - Designing Multi-Node System Architecture (Priority: P3)

**Goal**: Enable learners to design multi-node architectures using patterns

**Independent Test**: Learner designs multi-node system, justifies structure, implements prototype

**Dependencies**: User Stories 1, 2, 4 (needs pub-sub, messages, debugging knowledge)

**Tasks**:

- [ ] T045 [P] [US5] Write section 1.10.1: Design Principles in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain single responsibility principle
  - Describe loose coupling via topics
  - Discuss reusability and testability
  - List criteria for splitting functionality
  - Target: 150 words
  - FR-057

- [ ] T046 [P] [US5] Write section 1.10.2: Design Patterns in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Explain Pipeline pattern (A → B → C) with diagram
  - Explain Hierarchical pattern (layered) with diagram
  - Mention Star and Peer-to-peer as advanced
  - Include pros/cons from research.md Section 5
  - Target: 250 words
  - FR-058, FR-059

- [ ] T047 [P] [US5] Create Mermaid graph for Pipeline pattern and save to docs/module-01-robotic-nervous-system/_assets/diagrams/pipeline-pattern.mmd
  - Show Camera → Processor → Detector flow
  - Embed in section 1.10.2

- [ ] T048 [P] [US5] Create Mermaid graph for Hierarchical pattern and save to docs/module-01-robotic-nervous-system/_assets/diagrams/hierarchical-pattern.mmd
  - Show Perception → Planning → Control layers
  - Embed in section 1.10.2

- [ ] T049 [P] [US5] Write section 1.10.3: Anti-Patterns to Avoid in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - List circular dependencies, monolithic node, topic storm, tight coupling
  - Provide solutions for each
  - Target: 100 words
  - FR-060, FR-061

---

## Phase 8: User Story 6 - Quick Reference (Priority: P3)

**Goal**: Provide quick reference for experienced developers

**Independent Test**: Developer finds specific information within 1 minute using summary

**Dependencies**: All previous user stories (summary consolidates all content)

**Tasks**:

- [ ] T050 [P] [US6] Write section 2: Summary - Core Concepts Recap in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Bullet-point recap of nodes, topics, publishers, subscribers, messages, QoS, lifecycle
  - Target: 200 words
  - FR-003

- [ ] T051 [P] [US6] Write section 2: Summary - Command Reference in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - List all ros2 node, ros2 topic, ros2 interface commands with syntax
  - Include colcon build, source, ros2 run commands
  - Target: 200 words
  - FR-003

- [ ] T052 [P] [US6] Write section 2: Summary - Code Templates in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Python Publisher template
  - Python Subscriber template
  - QoS configuration examples
  - Target: 200 words
  - FR-003

- [ ] T053 [P] [US6] Write section 2: Summary - QoS Quick Reference in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Table mapping use cases to QoS profiles (from research.md Section 2)
  - Python QoS example code
  - Target: 150 words
  - FR-003

---

## Phase 9: Cross-Cutting Content & Polish

**Goal**: Complete remaining sections and polish chapter

**Dependencies**: All user stories complete

**Tasks**:

- [ ] T054 [P] Write section 1.11: Hands-On Exercises - Exercise 1 in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Objective: Create first pub-sub system
  - Steps: Create package, write publisher, write subscriber, build, run, verify
  - Expected outcome, difficulty (Beginner), time (20 min)
  - FR-062

- [ ] T055 [P] Write section 1.12: Troubleshooting in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Error 1: No communication (causes, debug, solution)
  - Error 2: Custom message not found
  - Error 3: Node crashes
  - Use content from research.md Section 6
  - Target: 300 words
  - FR-069

- [ ] T056 [P] Write section 1.13: Key Takeaways in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Bullet-point summary of major lessons
  - Target: 200 words
  - FR-068

- [ ] T057 [P] Create Multi-Node Communication diagram and save to docs/module-01-robotic-nervous-system/_assets/diagrams/multi-node-communication.mmd
  - Show complex multi-node system
  - Use for section 1.10 or 1.3
  - FR-064

- [ ] T058 Review complete chapter against quality checklist from quickstart.md
  - Technical terms defined on first use? (FR-065)
  - Code examples tested? (FR-037)
  - Expected outputs provided? (FR-039)
  - Diagrams support understanding? (FR-064)
  - Exercises reinforce concepts? (FR-062, FR-063)
  - Summary section complete? (FR-003)

- [ ] T059 Validate markdown formatting and syntax highlighting in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Code blocks use ```python, ```cpp, ```bash syntax
  - Commands in code blocks for easy copying
  - Front matter properly formatted
  - FR-070, FR-071, FR-072, FR-073

- [ ] T060 Add alt text to all diagrams in docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Mermaid diagrams have descriptive captions
  - Screenshots have alt attributes
  - FR-074

- [ ] T061 Perform final word count check on docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Full Lesson: 5000-7000 words (target met?)
  - Summary: 500-800 words (target met?)
  - Adjust if significantly over/under

- [ ] T062 Final constitution compliance review of docs/module-01-robotic-nervous-system/02-nodes-and-topics.md
  - Content-First: No filler, serves learning objectives
  - Simplicity & Clarity: Beginner-friendly, progressive complexity
  - Practical Application: Code tested, exercises included
  - All 7 constitution principles verified

---

## Task Summary

**Total Tasks**: 62
- **Setup**: 4 tasks (T001-T004)
- **User Story 1** (P1): 20 tasks (T005-T024)
- **User Story 2** (P1): 7 tasks (T025-T031)
- **User Story 3** (P2): 8 tasks (T032-T039)
- **User Story 4** (P2): 5 tasks (T040-T044)
- **User Story 5** (P3): 5 tasks (T045-T049)
- **User Story 6** (P3): 4 tasks (T050-T053)
- **Polish**: 9 tasks (T054-T062)

**Parallelizable Tasks**: 42 tasks marked with [P]

**Critical Path**: T001 → T005 → T017 → T020 → T023 → T024 (User Story 1 core)

---

## Dependencies & Execution Order

### User Story Dependencies

```
Phase 1: Setup
    ↓
Phase 3: User Story 1 (P1) ← MVP - Can ship after this
    ├→ Phase 4: User Story 2 (P1) ← Extends US1 with messages
    ├→ Phase 5: User Story 3 (P2) ← Extends US1 with QoS
    ├→ Phase 6: User Story 4 (P2) ← Extends US1 with debugging
    └→ Phase 7: User Story 5 (P3) ← Requires US1+US2+US4
        └→ Phase 8: User Story 6 (P3) ← Summarizes all
            └→ Phase 9: Polish ← Final touches
```

### User Story Independence

- **US1**: Independent (can complete alone) ✅ MVP
- **US2**: Depends on US1 (needs pub-sub foundation)
- **US3**: Depends on US1 (needs pub-sub foundation)
- **US4**: Depends on US1 (needs nodes/topics running)
- **US5**: Depends on US1, US2, US4 (needs full knowledge)
- **US6**: Depends on all (summarizes everything)

### Parallel Execution Opportunities

**Within User Story 1 (can execute in parallel after T005)**:
- T006-T011 (Node Architecture subsections)
- T012-T016 (Topics & Communication subsections)
- T018, T019 (C++ Publisher, Package Setup)
- T021, T022 (C++ Subscriber, Callbacks)

**Across User Stories (after US1 complete)**:
- US2, US3, US4 can be executed in parallel (all depend only on US1)

**Example Parallel Execution**:
1. Complete Setup (T001-T004)
2. Write US1 core sections in parallel: T006-T016
3. Write US1 code examples: T017 → T020 → T023 (sequential for testing)
4. Test US1: T024
5. Execute US2 (T025-T031), US3 (T032-T039), US4 (T040-T044) in parallel
6. Execute US5 (T045-T049) after US2+US4 complete
7. Execute US6 (T050-T053) and Polish (T054-T062) in parallel

---

## MVP Scope

**Minimum Viable Product**: Complete User Story 1 (Phase 3)

**MVP Deliverable**: Learners can understand node architecture, create working pub-sub nodes, and observe communication

**MVP Tasks**: T001-T024 (24 tasks)

**MVP Validation**:
- [ ] Learner reads sections 1.1-1.7
- [ ] Learner executes Python publisher example successfully
- [ ] Learner executes Python subscriber example successfully
- [ ] Learner runs both simultaneously and observes communication
- [ ] Learner modifies examples (change message, topic name) successfully

**Post-MVP Increments**:
1. **Increment 2**: Add US2 (custom messages) - 7 tasks
2. **Increment 3**: Add US3 (QoS policies) - 8 tasks
3. **Increment 4**: Add US4 (debugging tools) - 5 tasks
4. **Increment 5**: Add US5 (system design) - 5 tasks
5. **Increment 6**: Add US6 (summary) + Polish - 13 tasks

---

## Implementation Strategy

### Recommended Approach

1. **Start with MVP** (US1): Deliver core value first
2. **Test as you go**: Execute T024 after completing US1 code examples
3. **Parallelize when possible**: Use [P] markers to identify opportunities
4. **Independent testing**: Each user story should be independently testable
5. **Incremental delivery**: Ship US1, then add US2, then US3, etc.

### Quality Gates

After each user story phase:
- [ ] All code examples tested in ROS 2 Humble (FR-037)
- [ ] Expected outputs verified (FR-039)
- [ ] Diagrams created and embedded (FR-064)
- [ ] Technical terms defined (FR-065)
- [ ] Constitution principles maintained

### Success Criteria

**Complete when**:
- All 62 tasks marked as done
- All 15 success criteria from spec.md validated
- Chapter word count: 5000-7000 (Full Lesson) + 500-800 (Summary)
- All code examples execute without errors in ROS 2 Humble
- All diagrams render correctly in Docusaurus
- Constitution compliance review passed

---

## Notes

- **File Paths**: All tasks include specific file paths for clarity
- **Parallelization**: 42 tasks marked [P] can be executed in parallel
- **Story Labels**: [US1]-[US6] map to user stories from spec.md
- **Dependencies**: Clearly marked to enable proper sequencing
- **Independent Testing**: Each user story has independent test criteria
- **Beginner Focus**: All content maintains beginner-friendly approach
- **Code Testing**: Mandatory testing tasks after code sections (T024, T039)
- **Progressive Disclosure**: Content complexity increases gradually (US1 → US5)

**Ready for Implementation!** Follow task order, leverage parallelization, and test frequently.
