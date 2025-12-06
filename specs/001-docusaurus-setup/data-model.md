# Data Model: Docusaurus Book Structure

**Feature**: 001-docusaurus-setup
**Date**: 2025-12-03
**Phase**: 1 - Design

## Overview

This document defines the data structures and organizational patterns for book content in Docusaurus. Unlike traditional software applications with databases, this "data model" describes the file-based content structure, metadata schemas, and relationships between book components.

## Core Entities

### 1. Chapter

A chapter is the primary content unit of the book, represented as a single markdown file.

**File Representation**: `.md` file in the `docs/` directory

**Structure**:
```markdown
---
[Frontmatter: Chapter Metadata]
---

# Chapter Title

[Chapter Content: Markdown body]
```

**Attributes** (via Frontmatter):

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `id` | string | No | filename | Unique identifier for cross-referencing |
| `title` | string | No | first H1 | Full chapter title displayed on page |
| `sidebar_label` | string | No | title | Shortened label for sidebar navigation |
| `sidebar_position` | number | No | alphabetical | Explicit ordering within part |
| `description` | string | No | excerpt | SEO description and preview text |
| `slug` | string | No | file path | Custom URL path (e.g., `/chapter-name`) |
| `tags` | string[] | No | [] | Topic tags for categorization |
| `draft` | boolean | No | false | Hide from production if true |
| `hide_title` | boolean | No | false | Hide title at top of page |
| `hide_table_of_contents` | boolean | No | false | Hide right-side TOC |
| `custom_edit_url` | string\|null | No | computed | Override or disable edit link |

**Content Body**:
- Markdown formatted text
- Supports MDX (embedded React components if needed)
- Heading levels: H1 (title), H2-H6 (sections/subsections)
- Images referenced relative to `static/` directory
- Internal links to other chapters via doc IDs or slugs

**Validation Rules**:
- Filename must be valid filesystem name (no special chars except `-`, `_`)
- At least one heading (H1) should exist
- `id` must be unique across all chapters
- `slug` must be unique across all chapters
- Tags must match predefined tag set (if using tag file)

**Example**:
```markdown
---
id: getting-started
title: "Getting Started with Your Book"
sidebar_label: "Getting Started"
sidebar_position: 1
description: "An introduction to setting up and writing your book"
tags: [introduction, setup]
---

# Getting Started with Your Book

This chapter explains how to begin writing your book...

## Setting Up Your Environment

First, install the necessary tools...
```

### 2. Part

A part is a high-level grouping of related chapters, represented as a directory containing chapters.

**File Representation**: Directory in `docs/` with `_category_.json` metadata file

**Directory Structure**:
```text
docs/
└── part-01-foundations/
    ├── _category_.json
    ├── 01-chapter-one.md
    ├── 02-chapter-two.md
    └── 03-chapter-three.md
```

**Attributes** (via `_category_.json`):

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `label` | string | Yes | folder name | Display name in sidebar (e.g., "Part I: Foundations") |
| `position` | number | No | alphabetical | Order relative to other parts |
| `collapsible` | boolean | No | true | Allow expanding/collapsing in sidebar |
| `collapsed` | boolean | No | false | Initial collapsed state |
| `link` | object\|null | No | null | Optional landing page for the part |
| `customProps` | object | No | {} | Custom metadata for extensions |

**Example `_category_.json`**:
```json
{
  "label": "Part I: Foundations",
  "position": 1,
  "collapsible": true,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "title": "Part I: Foundations",
    "description": "Learn the fundamental concepts covered in this part."
  }
}
```

**Validation Rules**:
- Directory name should follow convention: `part-##-descriptive-name`
- Must contain at least one chapter (`.md` file)
- `_category_.json` must be valid JSON
- `position` should be unique (or follow numbering convention)

### 3. Section

A section is a subsection within a chapter, defined by heading levels (H2, H3, H4, etc.).

**File Representation**: Markdown headings within chapter file

**Structure**:
```markdown
## Section Title (H2)

Section content...

### Subsection Title (H3)

Subsection content...

#### Sub-subsection Title (H4)

More detailed content...
```

**Attributes**:
- **Level**: Heading level (H2-H6)
- **Title**: Text content of heading
- **ID**: Auto-generated slug from title (for anchor links)

**Automatic Features**:
- **Table of Contents**: Right sidebar automatically generated from H2-H3 headings
- **Anchor Links**: Each heading gets an anchor ID for deep linking
- **Navigation**: Headings appear in on-page navigation

**Validation Rules**:
- Heading hierarchy should be logical (don't skip levels: H2 → H4)
- Avoid duplicate heading text within same chapter (creates ID conflicts)
- Keep heading text concise for TOC readability

**Example**:
```markdown
## Understanding the Basics

This section covers fundamental concepts.

### Core Principles

Here are the key principles...

### Common Patterns

Let's explore common patterns...

## Advanced Topics

This section dives deeper...
```

### 4. Navigation Structure

The navigation structure defines the sidebar menu and reading order, combining parts and chapters.

**File Representation**: `sidebars.js` configuration file (root directory)

**Structure** (Autogenerated):
```javascript
module.exports = {
  bookSidebar: [
    {
      type: 'autogenerated',
      dirName: '.' // Auto-generate from docs/ folder
    }
  ]
};
```

**Structure** (Manual):
```javascript
module.exports = {
  bookSidebar: [
    'intro', // Root-level document
    {
      type: 'category',
      label: 'Part I: Foundations',
      items: [
        'part-01/chapter-01',
        'part-01/chapter-02',
        'part-01/chapter-03'
      ]
    },
    {
      type: 'category',
      label: 'Part II: Advanced Topics',
      items: [
        'part-02/chapter-04',
        'part-02/chapter-05'
      ]
    },
    {
      type: 'category',
      label: 'Appendix',
      items: ['appendix/glossary', 'appendix/references']
    }
  ]
};
```

**Generated Hierarchy**:
```text
Introduction (intro.md)
├─ Part I: Foundations
│  ├─ Chapter 1
│  ├─ Chapter 2
│  └─ Chapter 3
├─ Part II: Advanced Topics
│  ├─ Chapter 4
│  └─ Chapter 5
└─ Appendix
   ├─ Glossary
   └─ References
```

**Validation Rules**:
- All document IDs in sidebar must exist as files
- Category labels should match part names
- No circular references
- Sidebar structure should reflect logical reading order

### 5. Content Asset

Supporting files like images, diagrams, code samples, or downloadable resources.

**File Representation**: Files in `static/` directory

**Directory Structure**:
```text
static/
├── img/
│   ├── part-01/
│   │   ├── diagram-01.png
│   │   └── screenshot-01.png
│   └── part-02/
│       └── chart-01.svg
└── downloads/
    ├── sample-code.zip
    └── reference-sheet.pdf
```

**Reference in Markdown**:
```markdown
![Diagram showing workflow](/img/part-01/diagram-01.png)

Download: [Sample Code](/downloads/sample-code.zip)
```

**Attributes**:
- **Path**: Relative path from `static/` directory
- **Type**: File extension (png, jpg, svg, pdf, zip, etc.)
- **Alt Text**: Description for accessibility (images)

**Validation Rules**:
- Use web-friendly formats (PNG, JPG, SVG for images)
- Keep file sizes reasonable (<1MB per image ideal)
- Use descriptive filenames
- Organize by part or chapter for maintainability

## Relationships

```text
Book
├── Parts (0..N)
│   └── Chapters (1..N)
│       ├── Sections (1..N) [H2-H6 headings]
│       └── Assets (0..N) [Images, files]
└── Standalone Chapters (0..N) [e.g., intro, appendix items]
```

**Relationship Rules**:
- A Chapter belongs to zero or one Part
- A Part contains one or more Chapters
- A Chapter contains one or more Sections (via headings)
- Assets are referenced by Chapters but stored independently

## Directory Layout Example

```text
book-project/
├── docs/                           # Book content root
│   ├── intro.md                    # Introduction (standalone)
│   ├── part-01-foundations/        # Part I
│   │   ├── _category_.json
│   │   ├── 01-getting-started.md
│   │   ├── 02-basic-concepts.md
│   │   └── 03-first-steps.md
│   ├── part-02-advanced/           # Part II
│   │   ├── _category_.json
│   │   ├── 01-advanced-techniques.md
│   │   └── 02-best-practices.md
│   └── appendix/                   # Appendix
│       ├── _category_.json
│       ├── glossary.md
│       └── references.md
├── static/                         # Assets
│   └── img/
│       ├── part-01/
│       │   └── diagram-01.png
│       └── part-02/
│           └── chart-01.svg
├── sidebars.js                     # Navigation config
└── docusaurus.config.js            # Main config
```

## Metadata Flow

```text
Chapter File (intro.md)
│
├─→ Frontmatter
│   ├─→ title → Page Title
│   ├─→ sidebar_label → Sidebar Display
│   ├─→ description → SEO Meta Tag
│   ├─→ slug → URL Path
│   └─→ tags → Tag System
│
├─→ First H1 Heading → Fallback Title
│
├─→ H2-H3 Headings → Table of Contents
│
└─→ Content Body → Page Content
```

## File Naming Conventions

| Entity | Convention | Example |
|--------|------------|---------|
| Part directory | `part-##-descriptive-name` | `part-01-foundations` |
| Chapter file | `##-descriptive-title.md` | `01-getting-started.md` |
| Standalone chapter | `semantic-name.md` | `intro.md`, `conclusion.md` |
| Category metadata | `_category_.json` | (fixed name) |
| Image asset | `descriptive-name.ext` | `workflow-diagram.png` |
| Download asset | `descriptive-name.ext` | `sample-code.zip` |

**Rationale**:
- Numbers ensure predictable ordering
- Descriptive names improve maintainability
- Consistent patterns reduce configuration needs

## State Management

Docusaurus is a static site generator with no runtime state. All "state" is captured in:

1. **Source Files**: Markdown content, configuration files
2. **Build Artifacts**: Generated HTML/JS in `build/` directory
3. **Version Control**: Git tracks all changes to source files

**Chapter Lifecycle**:
1. **Draft**: Create `.md` file, set `draft: true` in frontmatter
2. **Review**: Edit content, iterate on structure
3. **Publish**: Remove `draft: true` or set to `false`
4. **Update**: Edit file, commit changes
5. **Archive**: Move to archive folder or add `draft: true`

## Constraints

- **File System Limits**: Chapter filename max 255 characters
- **URL Limits**: Slug should be <100 characters for SEO
- **Performance**: Keep chapters <10,000 words for optimal load time
- **Structure Depth**: Max 3 levels of nesting (Part → Chapter → Section) for UX
- **Asset Size**: Keep images <1MB, total static assets <50MB

## Validation Checklist

- [ ] All chapters have unique IDs
- [ ] All chapter files have valid frontmatter YAML
- [ ] All parts have `_category_.json` with label
- [ ] All internal links reference existing doc IDs
- [ ] All image paths resolve to files in `static/`
- [ ] No duplicate slugs across chapters
- [ ] Heading hierarchy is logical (no skipped levels)
- [ ] File/folder names follow naming conventions
