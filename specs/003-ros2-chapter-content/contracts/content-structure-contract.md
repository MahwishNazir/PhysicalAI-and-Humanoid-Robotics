# Content Structure Contract

**Feature**: 003-ros2-chapter-content
**Version**: 1.0.0
**Date**: 2025-12-04

## Purpose

This contract defines the mandatory structure and organization requirements for Chapter 1 "What is ROS 2?" to ensure consistency, quality, and educational effectiveness.

## Document Structure Contract

### Mandatory Sections

All implementations MUST include these sections in order:

1. **Frontmatter Block** (YAML)
2. **Full Lesson Section**
3. **Summary Section**

### Section 1: Frontmatter

**Contract**:
```yaml
---
title: "What is ROS 2?"                    # REQUIRED: Exact or close variation
sidebar_label: "What is ROS 2"             # REQUIRED: Sidebar text
sidebar_position: 1                         # REQUIRED: First chapter
description: "Learn ROS 2 fundamentals..."  # REQUIRED: SEO description (100-160 chars)
tags: [ros2, introduction, fundamentals]    # REQUIRED: Min 3 tags
keywords: [ROS 2, Robot Operating System]   # REQUIRED: Min 5 keywords
difficulty: beginner                        # REQUIRED: Must be "beginner"
estimated_time: "45-60 minutes"            # REQUIRED: Format "XX-YY minutes"
prerequisites: []                          # OPTIONAL: Empty array acceptable
---
```

**Validation**:
- All REQUIRED fields present
- `difficulty` = "beginner" (not negotiable)
- `sidebar_position` = 1
- `tags` array has 3+ elements
- `keywords` array has 5+ elements

### Section 2: Full Lesson

**Contract Structure**:
```markdown
## Full Lesson

### What is ROS 2?
[Comprehensive explanation: definition, purpose, history]
- Word count: 400-600 words
- MUST include: Definition, history overview, current state

### Why ROS 2 Matters
[Real-world context and applications]
- Word count: 300-500 words
- MUST include: Industry examples, advantages

### Core Concepts

#### Nodes
[Explanation of nodes as computational units]
- Word count: 300-400 words
- MUST include: Node definition, purpose, examples
- MUST include: Code example (publisher OR subscriber)

#### Topics and Publish-Subscribe
[Communication pattern explanation]
- Word count: 400-600 words
- MUST include: Pub-sub pattern, topic definition
- MUST include: Diagram (mermaid or image)
- MUST include: Code example (publisher AND subscriber)

#### Services
[Request-response communication]
- Word count: 250-350 words
- MUST include: Service definition, use cases
- SHOULD include: Code example

#### Actions
[Long-running task communication]
- Word count: 250-350 words
- MUST include: Action definition, difference from services
- MAY include: Code example

### Understanding DDS
[Data Distribution Service explanation]
- Word count: 300-500 words
- MUST include: DDS definition, role in ROS 2, middleware concept
- SHOULD include: Diagram showing middleware layer

### Quality of Service (QoS)
[QoS policies explanation]
- Word count: 400-600 words
- MUST include: QoS definition, reliability levels, durability
- MUST include: Table comparing QoS profiles

### ROS 2 vs ROS 1
[Comparison section]
- Word count: 400-600 words
- MUST include: Comparison table
- MUST include: Migration considerations
- Columns: Feature, ROS 1, ROS 2

### ROS 2 Tools
[CLI and GUI tools]
- Word count: 400-600 words
- MUST include: ros2 CLI commands
- MUST include: rqt overview
- MUST include: RViz2 mention
- MUST include: Code blocks with command examples

### Your First ROS 2 System
[Hands-on walkthrough]
- Word count: 400-600 words
- MUST include: Complete publisher code example
- MUST include: Complete subscriber code example
- MUST include: Instructions to run both
- MUST include: Expected output description
```

**Total Full Lesson Word Count**: 4000-5500 words

**Validation**:
- All subsections present
- Word counts within ranges
- Minimum code examples: 3 (publisher, subscriber, CLI commands)
- Minimum diagrams: 1
- Minimum tables: 2 (QoS comparison, ROS 1 vs 2)

### Section 3: Summary

**Contract Structure**:
```markdown
## Summary

### Key Concepts
[Bulleted list of main ideas]
- 15-25 bullet points
- Each bullet: 1-2 sentences max
- Categories: What ROS 2 is, Communication patterns, Tools

### Essential Commands
[Table of common CLI commands]
- Minimum 10 commands
- Format: | Command | Purpose |

### Quick Reference
[Links and resources]
- Link to next chapter
- Link to official ROS 2 docs
- Link to exercises/solutions

### Recap
[1-2 paragraph synthesis]
- 100-200 words
- Ties concepts together
```

**Total Summary Word Count**: 500-800 words

**Validation**:
- Fits on 1-2 pages when rendered
- Key Concepts has 15-25 bullets
- Commands table has 10+ rows
- All links valid

## Code Example Contract

### Minimum Code Requirements

**Publisher Example**:
```python
#!/usr/bin/env python3
# REQUIRED: Shebang line
# REQUIRED: Module docstring
# REQUIRED: Inline comments (min 1 per 10 lines)
# REQUIRED: Follows PEP 8
# REQUIRED: Uses std_msgs/String or similar simple type
# REQUIRED: Tested in ROS 2 Humble
# REQUIRED: Includes main() function
# REQUIRED: Includes if __name__ == '__main__' guard
```

**Subscriber Example**:
```python
#!/usr/bin/env python3
# Same requirements as Publisher
# MUST demonstrate callback function
# MUST log received messages
```

**CLI Commands Example**:
```bash
# REQUIRED: Show actual commands, not placeholders
# REQUIRED: Include expected output (commented)
# REQUIRED: Minimum 5 commands shown
```

### Code Quality Standards

- **Naming**: Descriptive variable names (not `x`, `y`, `data1`)
- **Comments**: Explain WHY, not WHAT (obvious code doesn't need comments)
- **Imports**: Grouped logically (standard lib, third-party, local)
- **Length**: Individual examples < 50 lines (split if longer)
- **Runnable**: Must execute without modification

## Multimedia Contract

### Video Embeddings

**Requirements**:
- **Maximum**: 3 videos per chapter
- **Duration**: Each video < 15 minutes
- **Source**: YouTube, Vimeo, or official ROS hosting
- **Recency**: Published within last 3 years
- **Quality**: 720p minimum resolution

**Format**:
```html
<iframe width="560" height="315"
  src="[VIDEO_URL]"
  title="[DESCRIPTIVE_TITLE]"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>
```

**Fallback**: Always provide direct link as alternative

### Presentations

**Requirements**:
- **Format**: Google Slides (preferred) or PDF
- **Accessibility**: Public view access, no login required
- **Content**: Architectural diagrams, concept visualizations
- **Text Alternative**: Bullet points summarizing key slides

### Diagrams

**Requirements**:
- **Format**: Mermaid (preferred) or SVG/PNG images
- **Clarity**: Readable at default size, no zooming required
- **Labels**: All nodes and connections labeled
- **Alt Text**: Descriptive alternative text for images

## Exercise Contract

### Minimum Exercise Requirements

**Required Exercises** (at least 3):

1. **Exercise 1**: Create a Simple Publisher
   - Difficulty: Beginner
   - Time: 10-15 minutes
   - Must include: Goal, Tasks, Validation checklist

2. **Exercise 2**: Create a Simple Subscriber
   - Difficulty: Beginner
   - Time: 10-15 minutes
   - Must include: Goal, Tasks, Validation checklist

3. **Exercise 3**: Publisher-Subscriber Pair
   - Difficulty: Intermediate
   - Time: 20-30 minutes
   - Must include: Goal, Tasks, Validation checklist

**Exercise Template Compliance**:
```markdown
### Exercise [N]: [Title]

**Difficulty**: [beginner|intermediate|advanced]
**Estimated Time**: [Duration]
**Goal**: [Clear objective statement]

**Prerequisites**:
- [Required knowledge]
- [Required setup]

**Tasks**:
1. [Step 1]
2. [Step 2]
...

**Validation Checklist**:
- [ ] [Success criterion 1]
- [ ] [Success criterion 2]

**Hints** (optional):
- [Helpful tip]

**Solution**: [Link to reference implementation]
```

## Accessibility Contract

### Text Alternatives

- **Images**: MUST have descriptive alt text
- **Videos**: MUST have transcript link or text summary
- **Diagrams**: MUST have explanatory paragraph
- **Code**: MUST have plain-language description

### Language Standards

- **Reading Level**: Undergraduate (Flesch-Kincaid 60-70)
- **Sentence Length**: Average < 25 words
- **Paragraph Length**: 3-5 sentences ideal
- **Active Voice**: Preferred over passive
- **Acronyms**: Defined on first use

## Validation Checklist

Before marking content as "complete", verify:

- [ ] Frontmatter has all required fields
- [ ] Full Lesson has all mandatory subsections
- [ ] Word count: 4500-6000 total
- [ ] Summary: 500-800 words
- [ ] Minimum 3 code examples
- [ ] All code tested in ROS 2 Humble
- [ ] Minimum 1 diagram
- [ ] Minimum 2 tables
- [ ] Minimum 3 exercises
- [ ] All multimedia has alt text/transcripts
- [ ] No broken links
- [ ] Markdown lints cleanly
- [ ] Reading time: 45-75 minutes

## Version History

- **1.0.0** (2025-12-04): Initial content structure contract
