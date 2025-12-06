# Tasks: Physical AI & Humanoid Robotics - Four Module Structure

**Input**: Design documents from `/specs/002-book-module-structure/`
**Prerequisites**: plan.md (complete), spec.md (complete), data-model.md (complete), contracts/ (complete)

**Tests**: Not requested in specification - no test tasks included

**Organization**: Tasks are grouped by user story (module) to enable independent implementation of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

This is a **documentation/book project** using Docusaurus:
- **Content**: `docs/` directory (markdown files)
- **Static assets**: `static/img/`, `static/downloads/`
- **Configuration**: `docusaurus.config.js`, `sidebars.js`
- **Source**: `src/` (Docusaurus customization)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing Docusaurus setup and create shared directory structure

- [X] T001 Verify Docusaurus installation and dev server works (npm start)
- [X] T002 [P] Create static asset directories: static/img/module-01/, static/img/module-02/, static/img/module-03/, static/img/module-04/
- [X] T003 [P] Create downloads directory: static/downloads/code-examples/
- [X] T004 [P] Create appendix directory structure: docs/appendix/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create appendix content that all modules will reference

**⚠️ CRITICAL**: Modules will reference glossary and setup guide - must be complete first

- [X] T005 Create appendix _category_.json in docs/appendix/ with position 5
- [X] T006 [P] Create glossary.md template in docs/appendix/ with Physical AI terminology structure
- [X] T007 [P] Create references.md template in docs/appendix/ with citation structure
- [X] T008 [P] Create setup-guide.md in docs/appendix/ with environment setup instructions (Python 3.8+, ROS 2, PyBullet, Gazebo)

**Checkpoint**: Foundation ready - module implementation can now begin in parallel

---

## Phase 3: User Story 1 - Module 1: The Robotic Nervous System (ROS 2) (Priority: P1) 🎯 MVP

**Goal**: Create Module 1 with 8 chapters on ROS 2 architecture. Chapter 1 includes full pilot content following 6-step pedagogical structure.

**Independent Test**: Module 1 should be fully navigable with Chapter 1 containing complete runnable content

### Directory and Metadata for User Story 1

- [X] T009 [US1] Create module directory: docs/module-01-robotic-nervous-system/
- [X] T010 [US1] Create module _category_.json in docs/module-01-robotic-nervous-system/ per schema (label: "Module 1: The Robotic Nervous System (ROS 2)", position: 1)
- [X] T011 [US1] Create module index.md in docs/module-01-robotic-nervous-system/ with module overview, learning objectives, prerequisites, technologies (ROS 2, Python, Gazebo/PyBullet)

### Chapter 1 (PILOT - Full Content) for User Story 1

- [X] T012 [US1] Create chapter file: docs/module-01-robotic-nervous-system/01-what-is-ros2.md with frontmatter per schema
- [X] T013 [US1] Write "Why This Matters" section in 01-what-is-ros2.md (intuitive motivation, real-world context)
- [X] T014 [US1] Write "The Big Picture" section in 01-what-is-ros2.md (plain-language explanation without jargon)
- [X] T015 [US1] Write "Technical Deep Dive" section in 01-what-is-ros2.md (formal ROS 2 definition, terminology)
- [X] T016 [US1] Create ROS 2 architecture diagram: static/img/module-01/ros2-architecture.svg or .png
- [X] T017 [US1] Write "Seeing It in Action" section in 01-what-is-ros2.md with diagram reference
- [X] T018 [US1] Create Python code example: static/downloads/code-examples/module-01/01-hello-ros2.py
- [X] T019 [US1] Write "Hands-On Code" section in 01-what-is-ros2.md with code example, inline comments
- [X] T020 [US1] Write "Try It Yourself" section in 01-what-is-ros2.md with practical simulation exercise
- [X] T021 [US1] Add Summary and Further Reading sections to 01-what-is-ros2.md
- [X] T022 [US1] Validate Chapter 1 against constitution standards (beginner accessibility, 1500-3000 words, all 6 sections present)

### Placeholder Chapters (2-8) for User Story 1

- [X] T023 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/02-nodes-and-topics.md with frontmatter only
- [X] T024 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/03-services-and-actions.md with frontmatter only
- [X] T025 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/04-robot-state-and-sensors.md with frontmatter only
- [X] T026 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/05-actuator-control.md with frontmatter only
- [X] T027 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/06-launch-files.md with frontmatter only
- [X] T028 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/07-simulation-setup.md with frontmatter only
- [X] T029 [P] [US1] Create placeholder chapter: docs/module-01-robotic-nervous-system/08-first-robot-system.md with frontmatter only

**Checkpoint**: Module 1 should be fully navigable with Chapter 1 pilot complete and validated

---

## Phase 4: User Story 2 - Module 2: Robot Perception and Sensors (Priority: P1)

**Goal**: Create Module 2 with 8 placeholder chapters on robot perception (sensors, vision, fusion)

**Independent Test**: Module 2 should be navigable with all 8 placeholder chapters

### Directory and Metadata for User Story 2

- [ ] T030 [US2] Create module directory: docs/module-02-robot-perception/
- [ ] T031 [US2] Create module _category_.json in docs/module-02-robot-perception/ per schema (label: "Module 2: Robot Perception and Sensors", position: 2)
- [ ] T032 [US2] Create module index.md in docs/module-02-robot-perception/ with module overview, learning objectives, prerequisites (Module 1), technologies (OpenCV, PCL, ROS 2 sensors)

### Placeholder Chapters (1-8) for User Story 2

- [ ] T033 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/01-introduction-to-sensors.md with frontmatter
- [ ] T034 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/02-camera-vision.md with frontmatter
- [ ] T035 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/03-depth-perception.md with frontmatter
- [ ] T036 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/04-imu.md with frontmatter
- [ ] T037 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/05-proprioception.md with frontmatter
- [ ] T038 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/06-sensor-fusion.md with frontmatter
- [ ] T039 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/07-computer-vision.md with frontmatter
- [ ] T040 [P] [US2] Create placeholder chapter: docs/module-02-robot-perception/08-perception-pipeline.md with frontmatter

**Checkpoint**: Module 2 should be fully navigable with all placeholder chapters

---

## Phase 5: User Story 3 - Module 3: Robot Motion and Control (Priority: P1)

**Goal**: Create Module 3 with 8 placeholder chapters on motion control (kinematics, dynamics, planning)

**Independent Test**: Module 3 should be navigable with all 8 placeholder chapters

### Directory and Metadata for User Story 3

- [ ] T041 [US3] Create module directory: docs/module-03-motion-control/
- [ ] T042 [US3] Create module _category_.json in docs/module-03-motion-control/ per schema (label: "Module 3: Robot Motion and Control", position: 3)
- [ ] T043 [US3] Create module index.md in docs/module-03-motion-control/ with module overview, learning objectives, prerequisites (Module 1, Module 2), technologies (MoveIt 2, ikpy, PyBullet)

### Placeholder Chapters (1-8) for User Story 3

- [ ] T044 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/01-kinematics-fundamentals.md with frontmatter
- [ ] T045 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/02-dynamics-physics.md with frontmatter
- [ ] T046 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/03-control-theory.md with frontmatter
- [ ] T047 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/04-trajectory-planning.md with frontmatter
- [ ] T048 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/05-manipulation-control.md with frontmatter
- [ ] T049 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/06-locomotion-fundamentals.md with frontmatter
- [ ] T050 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/07-whole-body-control.md with frontmatter
- [ ] T051 [P] [US3] Create placeholder chapter: docs/module-03-motion-control/08-motion-planning-project.md with frontmatter

**Checkpoint**: Module 3 should be fully navigable with all placeholder chapters

---

## Phase 6: User Story 4 - Module 4: Humanoid Robots and Integration (Priority: P1)

**Goal**: Create Module 4 with 8 placeholder chapters on humanoid integration (bipedal locomotion, HRI, sim-to-real)

**Independent Test**: Module 4 should be navigable with all 8 placeholder chapters

### Directory and Metadata for User Story 4

- [ ] T052 [US4] Create module directory: docs/module-04-humanoid-integration/
- [ ] T053 [US4] Create module _category_.json in docs/module-04-humanoid-integration/ per schema (label: "Module 4: Humanoid Robots and Integration", position: 4)
- [ ] T054 [US4] Create module index.md in docs/module-04-humanoid-integration/ with module overview, learning objectives, prerequisites (all previous modules), technologies (Isaac Sim, PyBullet, real humanoid platforms)

### Placeholder Chapters (1-8) for User Story 4

- [ ] T055 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/01-humanoid-anatomy.md with frontmatter
- [ ] T056 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/02-balance-stability.md with frontmatter
- [ ] T057 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/03-bipedal-locomotion.md with frontmatter
- [ ] T058 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/04-manipulation-dexterity.md with frontmatter
- [ ] T059 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/05-human-robot-interaction.md with frontmatter
- [ ] T060 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/06-behavior-arbitration.md with frontmatter
- [ ] T061 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/07-sim-to-real-transfer.md with frontmatter
- [ ] T062 [P] [US4] Create placeholder chapter: docs/module-04-humanoid-integration/08-capstone-project.md with frontmatter

**Checkpoint**: Module 4 should be fully navigable with all placeholder chapters

---

## Phase 7: User Story 5 - Update Navigation and Configuration (Priority: P2)

**Goal**: Update site configuration, remove old structure, and create book introduction

**Independent Test**: Site should display "Physical AI & Humanoid Robotics" title, intro should reflect new focus, old parts should be removed

### Configuration Updates for User Story 5

- [ ] T063 [US5] Update docusaurus.config.js title to "Physical AI & Humanoid Robotics"
- [ ] T064 [US5] Update docusaurus.config.js tagline to reflect Physical AI and Humanoid Robotics focus
- [ ] T065 [US5] Update docs/intro.md with Physical AI & Humanoid Robotics introduction (book goal, target audience, prerequisites, technology stack, how to use)

### Cleanup for User Story 5

- [ ] T066 [US5] Remove old structure: docs/part-01-introduction/ directory (if exists)
- [ ] T067 [US5] Remove old structure: docs/part-02-core-concepts/ directory (if exists)
- [ ] T068 [US5] Verify sidebars.js is configured for autogenerated sidebars or update manually if needed

### Glossary Population for User Story 5

- [ ] T069 [P] [US5] Add Physical AI terms to docs/appendix/glossary.md (at minimum: ROS 2, Node, Topic, Service, Action, Forward Kinematics, Inverse Kinematics, End-Effector, IMU, Proprioception, ZMP, Bipedal Locomotion, Sim-to-Real)
- [ ] T070 [P] [US5] Add references to docs/appendix/references.md (ROS 2 docs, Gazebo docs, PyBullet, research papers on humanoid robotics)

**Checkpoint**: Site should be fully configured with new Physical AI focus and all old structure removed

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Validation, documentation, and final touches

- [ ] T071 Run dev server and verify all modules are navigable (npm start)
- [ ] T072 Verify Module 1 Chapter 1 pilot content displays correctly with images and code
- [ ] T073 Run production build to check for errors (npm run build)
- [ ] T074 [P] Validate all _category_.json files against module-metadata-schema.json
- [ ] T075 [P] Validate all chapter frontmatter against chapter-frontmatter-schema.json
- [ ] T076 [P] Verify all image references in Module 1 Chapter 1 resolve correctly
- [ ] T077 [P] Verify all code download links work correctly
- [ ] T078 Check that glossary terms are alphabetically sorted
- [ ] T079 Verify references have proper citation format
- [ ] T080 Run constitution compliance check on Module 1 Chapter 1 (beginner accessibility, practical application, 6-step structure)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories 1-4 (Phases 3-6)**: All depend on Foundational phase completion
  - These 4 module phases can proceed in parallel (if staffed)
  - Or sequentially in order (Module 1 → 2 → 3 → 4)
- **User Story 5 (Phase 7)**: Depends on modules being created - should run after Phases 3-6
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (Module 1)**: Can start after Foundational (Phase 2) - No dependencies on other modules
- **User Story 2 (Module 2)**: Can start after Foundational (Phase 2) - References Module 1 as prerequisite but can be created in parallel
- **User Story 3 (Module 3)**: Can start after Foundational (Phase 2) - References Modules 1 & 2 as prerequisites but can be created in parallel
- **User Story 4 (Module 4)**: Can start after Foundational (Phase 2) - References all modules as prerequisites but can be created in parallel
- **User Story 5 (Configuration)**: Should run after modules are created (after Phases 3-6)

### Within Each User Story

- Directory and metadata tasks must complete before chapter creation
- Module index.md should be created after _category_.json
- Chapter 1 pilot content (US1 only) has sequential dependencies: file → sections → diagram → code → validation
- Placeholder chapters can all be created in parallel (marked [P])

### Parallel Opportunities

- **Phase 1 (Setup)**: T002, T003, T004 can run in parallel
- **Phase 2 (Foundational)**: T006, T007, T008 can run in parallel
- **Phase 3 (US1 Placeholders)**: T023-T029 can run in parallel (7 chapters)
- **Phase 4 (US2 All Chapters)**: T033-T040 can run in parallel (8 chapters)
- **Phase 5 (US3 All Chapters)**: T044-T051 can run in parallel (8 chapters)
- **Phase 6 (US4 All Chapters)**: T055-T062 can run in parallel (8 chapters)
- **Phase 7 (US5 Glossary)**: T069, T070 can run in parallel
- **Phase 8 (Polish)**: T074-T077 can run in parallel (validation tasks)
- **Across User Stories**: Once Phase 2 completes, Phases 3-6 (all 4 modules) can be worked on in parallel by different team members

---

## Parallel Example: User Story 1 (Module 1)

```bash
# After T011 (module index.md) completes, launch all placeholder chapters in parallel:
Task: T023 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/02-nodes-and-topics.md"
Task: T024 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/03-services-and-actions.md"
Task: T025 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/04-robot-state-and-sensors.md"
Task: T026 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/05-actuator-control.md"
Task: T027 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/06-launch-files.md"
Task: T028 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/07-simulation-setup.md"
Task: T029 - "Create placeholder chapter: docs/module-01-robotic-nervous-system/08-first-robot-system.md"
```

## Parallel Example: All Modules After Foundational Phase

```bash
# After Phase 2 completes, launch all 4 module phases in parallel:
Task: Phase 3 (US1) - "Module 1: ROS 2"
Task: Phase 4 (US2) - "Module 2: Perception"
Task: Phase 5 (US3) - "Module 3: Motion Control"
Task: Phase 6 (US4) - "Module 4: Humanoid Integration"
```

---

## Implementation Strategy

### MVP First (Module 1 Only with Pilot Chapter)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all modules)
3. Complete Phase 3: Module 1 (including Chapter 1 pilot)
4. **STOP and VALIDATE**: Test Module 1 Chapter 1 against constitution standards
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Module 1 with pilot → Test independently → Deploy/Demo (MVP!)
3. Add Module 2 → Test independently → Deploy/Demo
4. Add Module 3 → Test independently → Deploy/Demo
5. Add Module 4 → Test independently → Deploy/Demo
6. Update navigation and configuration → Final validation → Deploy
7. Each module adds value without breaking previous modules

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: Module 1 (US1) - includes pilot chapter authoring
   - Developer B: Module 2 (US2) - placeholder chapters
   - Developer C: Module 3 (US3) - placeholder chapters
   - Developer D: Module 4 (US4) - placeholder chapters
3. Developer E (or A after Module 1): Configuration updates (US5)
4. All team: Polish and validation
5. Modules complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label maps task to specific user story (US1-US5) for traceability
- Each module should be independently navigable and testable
- Module 1 Chapter 1 is the only chapter with full content (pilot) - all others are placeholders
- Constitution compliance gates apply to Module 1 Chapter 1 pilot content
- Commit after each logical task group (e.g., after creating a module's directory structure)
- Stop at any checkpoint to validate module independently
- Tests are not included as they were not requested in the specification

---

## Summary

- **Total Tasks**: 80 tasks
- **Task Count by User Story**:
  - Setup: 4 tasks
  - Foundational: 4 tasks
  - US1 (Module 1): 21 tasks (includes pilot content authoring)
  - US2 (Module 2): 11 tasks
  - US3 (Module 3): 11 tasks
  - US4 (Module 4): 11 tasks
  - US5 (Configuration): 8 tasks
  - Polish: 10 tasks
- **Parallel Opportunities**: 43 tasks marked [P] can run in parallel
- **Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (Module 1 with pilot chapter)
- **Critical Path**: Setup → Foundational → Module 1 pilot content authoring and validation
