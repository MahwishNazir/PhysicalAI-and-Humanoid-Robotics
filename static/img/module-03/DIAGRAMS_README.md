# Module 3 Diagram Style Guide

Consistent visual style for all Module 3 diagrams ensures professional appearance and enhances learning.

---

## Design Principles

1. **Clarity First**: Every diagram should be understandable without reading the text
2. **Consistent Style**: Use the same colors, fonts, and shapes across all diagrams
3. **Accessibility**: Include descriptive alt text and sufficient color contrast
4. **Scalability**: Create vector graphics (SVG) when possible for crisp rendering at all sizes

---

## Color Palette

### Primary Colors (NVIDIA Isaac & Robotics Theme)

```
NVIDIA Green:     #76B900  (Primary brand color, use for key elements)
Deep Blue:        #2C3E50  (Text, borders, containers)
Light Blue:       #3498DB  (Highlights, data flow)
Gray:             #95A5A6  (Secondary elements, backgrounds)
White:            #FFFFFF  (Backgrounds, text on dark)
Black:            #000000  (Text, strong outlines)
```

### Semantic Colors

```
Success/Active:   #27AE60  (Green - operational states)
Warning:          #F39C12  (Orange - caution, intermediate states)
Error/Stop:       #E74C3C  (Red - errors, stopped states)
Info:             #3498DB  (Blue - informational elements)
Neutral:          #7F8C8D  (Gray - inactive, disabled)
```

### Background Shades

```
Light Gray:       #ECF0F1  (Diagram backgrounds)
Dark Gray:        #34495E  (Section backgrounds)
Accent Teal:      #1ABC9C  (Callouts, highlights)
Accent Purple:    #9B59B6  (Alternative highlights)
```

---

## Typography

### Recommended Fonts

**Primary**: **Roboto** or **Open Sans** (clean, modern, readable)
**Monospace** (for code): **Roboto Mono** or **Fira Code**
**Fallback**: Arial, Helvetica, sans-serif

### Font Sizes

- **Title**: 24-28px (bold)
- **Headings**: 18-20px (semi-bold)
- **Labels**: 14-16px (regular)
- **Annotations**: 12-14px (regular or italic)
- **Code**: 12-14px (monospace)

### Text Guidelines

- Use **sentence case** for labels (not ALL CAPS unless acronyms)
- Keep labels concise (1-4 words ideal)
- Use dark text (#2C3E50) on light backgrounds
- Use white text (#FFFFFF) on dark backgrounds (#2C3E50)
- Ensure minimum 4.5:1 contrast ratio (WCAG AA standard)

---

## Shape Conventions

### Standard Shapes

| Shape | Meaning | Usage |
|-------|---------|-------|
| **Rectangle** | Component, module, entity | Use for systems, services, models |
| **Rounded Rectangle** | Process, function, action | Use for operations, algorithms |
| **Diamond** | Decision point | Use for conditional logic, branching |
| **Circle/Ellipse** | State, data | Use for configuration, variables |
| **Cylinder** | Database, storage | Use for data persistence |
| **Cloud** | External system | Use for third-party services, APIs |
| **Document** | File, specification | Use for config files, documentation |
| **Arrow** | Data flow, direction | Use for showing relationships, flow |

### Component Sizing

- **Large boxes**: 200-300px width for major components
- **Medium boxes**: 150-200px width for sub-components
- **Small boxes**: 100-150px width for minor elements
- **Padding**: Minimum 10px internal padding
- **Spacing**: 20-40px between elements

---

## Diagram Types & Templates

### 1. Architecture Diagrams

**Purpose**: Show system structure and component relationships

**Style Guidelines**:
- Use rectangles for components
- Use arrows for dependencies/data flow
- Group related components with light gray backgrounds
- Use NVIDIA Green (#76B900) for Isaac Sim components
- Use Deep Blue (#2C3E50) for ROS 2 components

**Example Elements**:
```
[Isaac Sim]  →  [Perception Module]  →  [Robot Controller]
 (Green)          (Light Blue)            (Deep Blue)
```

### 2. Workflow/Process Diagrams

**Purpose**: Show step-by-step processes or algorithms

**Style Guidelines**:
- Use rounded rectangles for process steps
- Use diamonds for decisions
- Use numbered labels (1, 2, 3...)
- Use consistent arrow styles (solid for primary flow, dashed for alternate paths)
- Color-code by stage: Start (Green), Process (Blue), End (Gray)

**Flow Direction**: Top-to-bottom or left-to-right (be consistent within one diagram)

### 3. Data Flow Diagrams

**Purpose**: Show how data moves through system

**Style Guidelines**:
- Use bold arrows (#3498DB) for data flow
- Label arrows with data type/format
- Use cylinders for data storage
- Use clouds for external data sources
- Show transformations with rounded rectangles

### 4. Component Interaction Diagrams

**Purpose**: Show how components communicate

**Style Guidelines**:
- Use sequence diagram style (time flows top-to-bottom)
- Use dashed arrows for return values
- Use solid arrows for function calls
- Number interactions sequentially
- Include brief descriptions under each step

### 5. Conceptual/Analogy Diagrams

**Purpose**: Explain complex concepts with simple visuals

**Style Guidelines**:
- Use simple, recognizable shapes
- Use fewer colors (2-3 max)
- Include short text annotations
- Make comparisons side-by-side
- Use icons when appropriate

---

## Technical Specifications

### File Formats

**Preferred**: SVG (Scalable Vector Graphics)
- Crisp at any zoom level
- Small file size
- Editable

**Acceptable**: PNG
- Minimum resolution: 800px width
- 72-96 DPI for web
- Transparent background when appropriate

**Not Recommended**: JPEG (lossy compression causes artifacts)

### Tool Recommendations

1. **Draw.io (diagrams.net)** - Free, web-based, exports to SVG/PNG
2. **Figma** - Professional, collaborative, free tier available
3. **Mermaid** - Code-based diagrams (markdown integration)
4. **Excalidraw** - Hand-drawn style (good for sketches)
5. **Lucidchart** - Professional diagramming (paid)

### Export Settings (Draw.io)

- **Format**: SVG or PNG
- **PNG Settings**:
  - Width: 1200px minimum
  - Transparent background: Yes (unless diagram requires white)
  - Border width: 10px padding
- **SVG Settings**:
  - Include embedded fonts: Yes
  - Remove text formatting: No

---

## Naming Conventions

### File Naming

Format: `{chapter-number}-{descriptive-name}.{ext}`

**Examples**:
- `01-isaac-ecosystem-diagram.png`
- `02-physics-configuration.svg`
- `04-slam-problem-formulation.png`

**Rules**:
- Lowercase only
- Use hyphens (not underscores or spaces)
- Be descriptive (avoid generic names like "diagram1.png")
- Include chapter number prefix for organization

### Alt Text Writing

**Format**: `[Brief description of diagram content and purpose]`

**Examples**:
```markdown
![Isaac Sim ecosystem architecture showing relationships between Isaac Sim, Omniverse, USD, and ROS 2 components](./01-isaac-ecosystem-diagram.png)

![Installation workflow flowchart with decision points for Windows vs Linux and GPU compatibility checks](./01-installation-workflow.png)

![USD scene structure hierarchy showing layers, prims, and attributes with parent-child relationships](./01-usd-structure-example.png)
```

**Guidelines**:
- Describe what the diagram shows, not just its title
- Include key elements visible in the diagram
- Keep under 150 characters when possible
- Focus on informational content, not decorative details

---

## Checklist for Diagram Creation

Before finalizing any diagram:

- [ ] Uses color palette from this guide
- [ ] Font sizes are readable (minimum 12px)
- [ ] Text has sufficient contrast (4.5:1 minimum)
- [ ] Arrows clearly show direction
- [ ] All elements are labeled
- [ ] Legends included if colors have meaning
- [ ] File named according to convention
- [ ] Saved in correct directory (`static/img/module-03/XX-chapter/`)
- [ ] Alt text written and descriptive
- [ ] Diagram referenced in chapter markdown
- [ ] Tested at different zoom levels (if SVG)
- [ ] Transparent background if appropriate (PNG)

---

## Example: Architecture Diagram Template

```
┌─────────────────────────────────────────────────────────┐
│ Title: Isaac Sim Ecosystem Architecture                 │
│ (24px, bold, #2C3E50)                                   │
└─────────────────────────────────────────────────────────┘

┌──────────────────┐         ┌──────────────────┐
│  Isaac Sim       │◄────────┤  Omniverse       │
│  #76B900         │         │  #3498DB         │
│  (Component)     │         │  (Platform)      │
└────────┬─────────┘         └──────────────────┘
         │
         │ Exports
         ▼
┌──────────────────┐         ┌──────────────────┐
│  USD Files       │────────►│  ROS 2 Interface │
│  #95A5A6         │         │  #2C3E50         │
│  (Data Format)   │  Sends  │  (Control)       │
└──────────────────┘  to     └──────────────────┘

Legend:
• Green boxes: Isaac Sim components
• Blue boxes: Omniverse platform
• Gray boxes: Data formats
• Dark blue: Robot control systems
```

---

## Common Mistakes to Avoid

❌ **Don't**:
- Use too many colors (stick to palette)
- Make text too small (<12px)
- Use low-contrast color combinations
- Create overly complex diagrams (split into multiple if needed)
- Use raster images for diagrams (prefer vector)
- Forget alt text
- Use generic file names ("image1.png")

✅ **Do**:
- Keep diagrams simple and focused
- Use consistent styling across all diagrams
- Test readability at different sizes
- Include legends when needed
- Use meaningful labels
- Provide descriptive alt text
- Export in appropriate format (SVG preferred)

---

## Resources

**Color Contrast Checker**: https://webaim.org/resources/contrastchecker/
**Draw.io**: https://app.diagrams.net/
**Mermaid Live Editor**: https://mermaid.live/
**Figma**: https://www.figma.com/
**WCAG Guidelines**: https://www.w3.org/WAI/WCAG21/quickref/

---

Use this guide to create professional, accessible, and consistent diagrams for all Module 3 chapters.
