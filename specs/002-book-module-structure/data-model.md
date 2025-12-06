# Data Model: Physical AI & Humanoid Robotics Book Structure

**Feature**: 002-book-module-structure
**Phase**: 1 (Design & Contracts)
**Date**: 2025-12-04

## Overview

This document defines the entities, relationships, and validation rules for the 4-module Physical AI & Humanoid Robotics book structure. The data model is file-based (markdown content) with metadata stored in YAML frontmatter and JSON files.

---

## Entity: Module

**Description**: Top-level organizational unit representing a major learning area (e.g., ROS 2, Perception, Control, Integration).

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `id` | string | Yes | Format: `module-##-slug` | Unique module identifier (directory name) |
| `number` | integer | Yes | Range: 1-4 | Module sequence number |
| `title` | string | Yes | Max 60 chars | Full module title |
| `shortTitle` | string | Yes | Max 30 chars | Abbreviated title for navigation |
| `description` | string | Yes | Max 200 chars | One-sentence module description |
| `position` | integer | Yes | Range: 1-4 | Sidebar position (1 = first) |
| `collapsible` | boolean | Yes | Default: `true` | Whether sidebar can collapse |
| `collapsed` | boolean | Yes | Default: `false` | Initial collapsed state |
| `estimatedHours` | integer | No | Positive integer | Estimated reading/practice time |
| `prerequisites` | array[string] | No | Module IDs or "none" | Required prior knowledge |
| `learningObjectives` | array[string] | No | - | Key learning outcomes |
| `technologies` | array[string] | No | - | Tools/libraries used in module |

### File Representation

**Directory**: `docs/[module-id]/`

**Metadata File**: `docs/[module-id]/_category_.json`
```json
{
  "label": "Module 1: The Robotic Nervous System (ROS 2)",
  "position": 1,
  "collapsible": true,
  "collapsed": false,
  "link": {
    "type": "doc",
    "id": "module-01-robotic-nervous-system/index"
  },
  "description": "Learn robot system architecture and communication using ROS 2"
}
```

**Index File**: `docs/[module-id]/index.md`
```yaml
---
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_label: "Overview"
sidebar_position: 0
description: "Learn robot system architecture and communication using ROS 2"
---

# Module 1: The Robotic Nervous System (ROS 2)

[Module overview content...]
```

### Validation Rules

1. **Unique Module ID**: No two modules can have the same `id`
2. **Sequential Numbering**: Module numbers must be consecutive (1, 2, 3, 4)
3. **Position Matches Number**: `position` should equal `number`
4. **Title Length**: Full title ≤ 60 characters, short title ≤ 30 characters
5. **Description**: Must be a single sentence ending with period
6. **Prerequisites**: Referenced modules must exist (or "none")
7. **Directory Exists**: Directory `docs/[module-id]/` must exist
8. **Metadata Files**: Both `_category_.json` and `index.md` must be present

### Relationships

- **Contains**: 1 Module contains 8 Chapters (one-to-many)
- **Requires**: 1 Module may require 0+ other Modules as prerequisites (many-to-many)
- **Uses**: 1 Module references 0+ Technologies (many-to-many)

---

## Entity: Chapter

**Description**: Individual learning unit within a module covering a specific topic or concept.

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `id` | string | Yes | Format: `##-slug` | Chapter file name (without `.md`) |
| `moduleId` | string | Yes | Valid module ID | Parent module identifier |
| `number` | integer | Yes | Range: 1-8 | Chapter number within module |
| `title` | string | Yes | Max 60 chars | Full chapter title |
| `sidebarLabel` | string | Yes | Max 30 chars | Shortened label for sidebar |
| `sidebarPosition` | integer | Yes | Range: 1-8 | Position within module sidebar |
| `description` | string | Yes | Max 160 chars | One-sentence chapter summary |
| `tags` | array[string] | No | - | Categorization tags |
| `keywords` | array[string] | No | - | SEO keywords |
| `wordCount` | integer | No | Range: 1500-3000 | Approximate word count |
| `difficulty` | enum | No | `beginner`, `intermediate`, `advanced` | Technical difficulty level |
| `prerequisites` | array[string] | No | Chapter IDs or "none" | Required prior chapters |
| `isPilot` | boolean | No | Default: `false` | Whether chapter has full content (vs placeholder) |

### File Representation

**File**: `docs/[module-id]/[chapter-id].md`

**Example**: `docs/module-01-robotic-nervous-system/01-what-is-ros2.md`

```yaml
---
title: "What is ROS 2?"
sidebar_label: "What is ROS 2?"
sidebar_position: 1
description: "Introduction to the Robot Operating System 2 middleware"
tags: [ros2, robotics, middleware, architecture]
keywords: [ROS 2, robot operating system, nodes, topics, robotics middleware]
---

# What is ROS 2?

[Chapter content following 6-step structure...]
```

### Content Structure (6-Step Pedagogical Model)

Every chapter MUST include these sections (per constitution Principle VII):

1. **Section: Why This Matters (Intuitive Motivation)**
   - Real-world problem or application
   - Why this concept is important
   - Preview of what readers will learn

2. **Section: The Big Picture (Plain-Language Explanation)**
   - Concept explained without technical jargon
   - Analogies and everyday examples
   - Visual thinking aids

3. **Section: Technical Deep Dive (Formal Definition)**
   - Precise terminology introduction
   - Mathematical notation (if applicable)
   - Technical specifications

4. **Section: Seeing It in Action (Visual Diagram)**
   - Labeled diagram or visualization
   - Legend and annotations
   - Multiple perspectives if needed (2D/3D)

5. **Section: Hands-On Code (Code Example)**
   - Python code snippet (3.8+)
   - Inline comments explaining each line
   - Runnable in specified environment

6. **Section: Try It Yourself (Practical Exercise)**
   - Guided hands-on activity
   - Simulation task or coding challenge
   - Expected outcomes and troubleshooting

**Additional Sections** (optional):
- **Summary**: Key takeaways
- **Further Reading**: Links to papers, documentation, advanced topics
- **Common Pitfalls**: Mistakes to avoid
- **FAQs**: Frequently asked questions

### Validation Rules

1. **Unique Chapter ID**: No two chapters within a module can have same `id`
2. **Sequential Numbering**: Chapter numbers must be consecutive (1-8 within module)
3. **Position Matches Number**: `sidebarPosition` should equal `number`
4. **Title Length**: Full title ≤ 60 characters, sidebar label ≤ 30 characters
5. **Description**: Single sentence ≤ 160 characters (meta description limit)
6. **Word Count**: Pilot chapters must have 1500-3000 words
7. **6-Step Structure**: Pilot chapters must include all 6 required sections
8. **Code Validity**: Code examples must be syntactically correct Python 3.8+
9. **Diagram Presence**: Visual diagram section must reference at least one image
10. **Prerequisites**: Referenced chapters must exist
11. **File Naming**: File name must match pattern `##-slug.md` where `##` is two-digit number

### Relationships

- **Belongs To**: 1 Chapter belongs to exactly 1 Module (many-to-one)
- **References**: 1 Chapter may reference 0+ Assets (images, code files) (one-to-many)
- **Requires**: 1 Chapter may require 0+ other Chapters as prerequisites (many-to-many)
- **Tagged With**: 1 Chapter has 0+ Tags (many-to-many)

---

## Entity: Asset

**Description**: Static files (images, code examples, robot models) referenced by chapters.

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `filePath` | string | Yes | Relative to `static/` | Asset file path |
| `type` | enum | Yes | `image`, `code`, `robot-model`, `other` | Asset category |
| `format` | string | Yes | File extension | File format (png, svg, py, urdf, etc.) |
| `moduleId` | string | No | Valid module ID | Associated module (for organization) |
| `chapterIds` | array[string] | No | Valid chapter IDs | Chapters that reference this asset |
| `altText` | string | Conditional | Required for images | Accessibility alt text |
| `caption` | string | No | - | Display caption |
| `width` | integer | No | Pixels | Image width (if applicable) |
| `height` | integer | No | Pixels | Image height (if applicable) |
| `license` | string | No | SPDX identifier | Asset license (e.g., CC-BY-4.0) |

### File Organization

**Images**:
```
static/img/
├── module-01/
│   ├── node-graph.png
│   ├── topic-communication.svg
│   └── ros2-architecture.svg
├── module-02/
│   ├── sensor-placement.png
│   ├── camera-frames.svg
│   └── point-cloud-viz.png
├── module-03/
│   ├── kinematic-chain.svg
│   ├── control-loop.png
│   └── trajectory-plot.svg
└── module-04/
    ├── humanoid-anatomy.png
    ├── balance-zmp.svg
    └── gait-cycle.png
```

**Code Examples**:
```
static/downloads/code-examples/
├── module-01/
│   ├── 01-hello-ros2.py
│   ├── 02-publisher-example.py
│   └── ...
├── module-02/
│   ├── 01-camera-capture.py
│   ├── 02-lidar-processing.py
│   └── ...
└── ...
```

**Robot Models**:
```
static/downloads/robot-models/
├── simple-arm.urdf
├── mobile-robot.urdf
└── humanoid-full.urdf
```

### Validation Rules

1. **File Exists**: Asset file must exist at specified `filePath`
2. **Format Matches**: File extension must match `format` attribute
3. **Alt Text Required**: Images must have `altText` for accessibility
4. **Referenced Assets**: All assets referenced in chapters must be defined
5. **Image Size**: Images should be ≤ 2MB; SVG preferred for diagrams
6. **Code Syntax**: Code files must be syntactically valid
7. **License Specified**: Assets from external sources must have license attribution

### Relationships

- **Referenced By**: 1 Asset is referenced by 0+ Chapters (one-to-many)
- **Belongs To**: 1 Asset may be organized under 1 Module (many-to-one)

---

## Entity: Technology

**Description**: External software, libraries, platforms, or tools used in the book.

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `id` | string | Yes | Slug format | Unique technology identifier |
| `name` | string | Yes | - | Full technology name |
| `version` | string | No | Semantic version | Specific version used (if applicable) |
| `type` | enum | Yes | `language`, `library`, `framework`, `platform`, `tool` | Technology category |
| `description` | string | Yes | Max 200 chars | One-sentence description |
| `url` | string | No | Valid URL | Official website or documentation |
| `installRequired` | boolean | Yes | Default: `true` | Whether users need to install this |
| `moduleIds` | array[string] | No | Valid module IDs | Modules that use this technology |

### Examples

```yaml
technologies:
  - id: ros2-humble
    name: ROS 2 Humble
    version: "2023.1"
    type: framework
    description: "Robot Operating System 2 middleware for robot communication"
    url: "https://docs.ros.org/en/humble/"
    installRequired: true
    moduleIds: [module-01-robotic-nervous-system, module-02-robot-perception, module-03-motion-control, module-04-humanoid-integration]

  - id: pybullet
    name: PyBullet
    version: "3.2.6"
    type: library
    description: "Python physics simulation library for robotics"
    url: "https://pybullet.org/"
    installRequired: true
    moduleIds: [module-01-robotic-nervous-system, module-03-motion-control, module-04-humanoid-integration]

  - id: opencv
    name: OpenCV
    version: "4.8+"
    type: library
    description: "Computer vision library for image processing"
    url: "https://opencv.org/"
    installRequired: true
    moduleIds: [module-02-robot-perception]
```

### Validation Rules

1. **Unique ID**: No two technologies can have the same `id`
2. **Valid URL**: If URL provided, must be valid HTTPS URL
3. **Version Format**: Version should follow semantic versioning (X.Y.Z)
4. **Referenced Modules**: All `moduleIds` must reference existing modules

### Relationships

- **Used By**: 1 Technology is used by 0+ Modules (many-to-many)
- **Referenced In**: 1 Technology may be referenced in 0+ Chapters (many-to-many)

---

## Entity: Glossary Term

**Description**: Technical term defined in the appendix glossary.

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `term` | string | Yes | - | The technical term |
| `definition` | string | Yes | Max 500 chars | Plain-language definition |
| `aliases` | array[string] | No | - | Alternative names or abbreviations |
| `category` | enum | No | `robotics`, `ai`, `control`, `mathematics`, `software` | Term category |
| `firstUseChapter` | string | No | Valid chapter ID | Chapter where term is first defined |
| `relatedTerms` | array[string] | No | Valid terms | Cross-referenced related terms |
| `externalLink` | string | No | Valid URL | Link to external resource (Wikipedia, etc.) |

### Example

```yaml
term: Inverse Kinematics
definition: "The process of calculating the joint angles needed to position a robot's end-effector at a desired location in space."
aliases: [IK, inverse kinematic problem]
category: robotics
firstUseChapter: module-03-motion-control/01-kinematics-fundamentals
relatedTerms: [Forward Kinematics, End-Effector, Joint Space, Cartesian Space]
externalLink: "https://en.wikipedia.org/wiki/Inverse_kinematics"
```

### Validation Rules

1. **Unique Term**: No duplicate terms in glossary
2. **Definition Length**: Definition must be concise (≤ 500 characters)
3. **First Use Exists**: If `firstUseChapter` specified, chapter must exist
4. **Related Terms Exist**: All `relatedTerms` must be defined in glossary
5. **Alphabetical Order**: Glossary terms must be alphabetically sorted

### Relationships

- **Defined In**: 1 Term is first defined in exactly 1 Chapter (many-to-one)
- **Related To**: 1 Term may be related to 0+ other Terms (many-to-many)

---

## Entity: Reference

**Description**: External source (paper, book, documentation) cited in the book.

### Attributes

| Attribute | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| `id` | string | Yes | Format: `[author][year]` | Citation key |
| `type` | enum | Yes | `paper`, `book`, `documentation`, `website`, `platform` | Reference type |
| `title` | string | Yes | - | Full title |
| `authors` | array[string] | Conditional | Required for papers/books | Author names |
| `year` | integer | No | 1900-2025 | Publication year |
| `url` | string | No | Valid URL | Online link |
| `doi` | string | No | Valid DOI | Digital Object Identifier (for papers) |
| `isbn` | string | No | Valid ISBN | International Standard Book Number (for books) |
| `publisher` | string | No | - | Publisher name |
| `citedInChapters` | array[string] | No | Valid chapter IDs | Chapters that cite this reference |

### Example

```yaml
references:
  - id: lynch2017
    type: book
    title: "Modern Robotics: Mechanics, Planning, and Control"
    authors: [Kevin M. Lynch, Frank C. Park]
    year: 2017
    publisher: Cambridge University Press
    isbn: "978-1107156302"
    url: "http://hades.mech.northwestern.edu/index.php/Modern_Robotics"
    citedInChapters: [module-03-motion-control/01-kinematics-fundamentals, module-03-motion-control/02-dynamics-physics]

  - id: ros2-docs
    type: documentation
    title: "ROS 2 Documentation"
    year: 2024
    url: "https://docs.ros.org/"
    citedInChapters: [module-01-robotic-nervous-system/01-what-is-ros2, module-01-robotic-nervous-system/02-nodes-and-topics]
```

### Validation Rules

1. **Unique ID**: No duplicate reference IDs
2. **Required Fields by Type**:
   - Papers: `authors`, `year`, `doi` or `url`
   - Books: `authors`, `year`, `publisher`, `isbn` or `url`
   - Documentation: `url`
3. **Valid URLs**: All URLs must be valid and accessible
4. **Cited Chapters Exist**: All `citedInChapters` must reference existing chapters

### Relationships

- **Cited By**: 1 Reference is cited by 0+ Chapters (one-to-many)

---

## Aggregate: Book

**Description**: Root entity representing the entire book.

### Attributes

| Attribute | Type | Value |
|-----------|------|-------|
| `title` | string | "Physical AI & Humanoid Robotics" |
| `version` | string | "1.0.0" (semantic versioning) |
| `domain` | string | "AI Systems in the Physical World, Embodied Intelligence" |
| `targetAudience` | string | "Beginners with foundational AI knowledge" |
| `moduleCount` | integer | 4 |
| `chapterCount` | integer | 32 (8 per module) |
| `constitutionVersion` | string | "1.1.0" |

### Structure

```
Book
├── Module 1: ROS 2
│   ├── Chapter 1: What is ROS 2? (PILOT)
│   ├── Chapter 2: Nodes and Topics
│   ├── ... (8 chapters total)
├── Module 2: Perception
│   ├── Chapter 1: Introduction to Sensors
│   ├── ... (8 chapters total)
├── Module 3: Control
│   ├── Chapter 1: Kinematics Fundamentals
│   ├── ... (8 chapters total)
├── Module 4: Integration
│   ├── Chapter 1: Humanoid Anatomy
│   ├── ... (8 chapters total)
└── Appendix
    ├── Glossary
    ├── References
    └── Setup Guide
```

---

## State Transitions

### Chapter Lifecycle

```
[Draft] → [Review] → [Pilot] → [Published]
  ↓          ↓          ↓           ↓
(placeholder) → (content written) → (validated) → (final)
```

**States**:
1. **Draft**: Placeholder with frontmatter only
2. **Review**: Content written, awaiting validation
3. **Pilot**: Full 6-step content, tested against constitution standards
4. **Published**: Approved, final version

**Transitions**:
- Draft → Review: Author completes 6-step content
- Review → Pilot: Passes beginner comprehension test, code execution verification
- Pilot → Published: Final approval, ready for release

### Module Lifecycle

```
[Planned] → [In Progress] → [Complete] → [Published]
```

**States**:
1. **Planned**: Module metadata created, chapters are placeholders
2. **In Progress**: Some chapters have content
3. **Complete**: All 8 chapters have pilot content
4. **Published**: All chapters published

---

## Validation Summary

### Global Constraints

1. **Module Count**: Exactly 4 modules
2. **Chapters Per Module**: Exactly 8 chapters per module
3. **Constitution Compliance**: All content must pass 7 constitution principles
4. **6-Step Structure**: Every pilot chapter must have all 6 required sections
5. **Code Validity**: All Python code must be syntactically correct and runnable
6. **Image Alt Text**: All images must have accessibility alt text
7. **Glossary Completeness**: All technical terms used must be defined
8. **References**: All claims must have documented sources

### File System Constraints

1. **Directory Structure**: Must match defined hierarchy
2. **File Naming**: Must follow `##-slug.md` pattern for chapters
3. **Metadata Files**: `_category_.json` and `index.md` required for each module
4. **Asset Paths**: All referenced assets must exist in `static/`

---

## Data Model Diagram

```
Book
  │
  ├──> Module (4)
  │      │
  │      ├──> Chapter (8 per module)
  │      │      │
  │      │      ├──> Asset (images, code)
  │      │      └──> Reference (citations)
  │      │
  │      └──> Technology (ROS 2, PyBullet, etc.)
  │
  └──> Appendix
         ├──> Glossary Term
         ├──> Reference
         └──> Setup Guide
```

---

## Summary

This data model provides:
- **Structure**: 4 modules, 32 chapters, appendix
- **Validation**: Rules ensuring constitution compliance
- **Relationships**: Module → Chapter → Asset → Reference
- **Lifecycle**: Draft → Review → Pilot → Published
- **Extensibility**: Can add modules/chapters without breaking structure

**Next Step**: Generate contract files (JSON schemas, markdown templates) based on this data model.
