---
id: 001
title: Blank Book Setup
stage: constitution
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: User
command: /sp.constitution
labels: ["constitution", "initialization", "book-writing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

I have to write a book, subject and book tile isnt decided, i want to make a blank setup first.

## Response snapshot

Created initial Book Project Constitution (v1.0.0) with six core principles:

1. **Content-First**: Every element serves reader understanding and engagement
2. **Incremental Development**: Build iteratively (outline → draft → revision → polish)
3. **Version Control**: Track all content changes through git
4. **Consistency**: Maintain unified voice, tone, terminology, and formatting
5. **Research-Backed**: All claims must be verifiable with documented sources
6. **Simplicity and Clarity**: Prefer simple explanations, define jargon, prioritize accessibility

Additional sections defined:
- **Content Standards**: Structure, formatting, length targets, asset organization, metadata
- **Review Process**: Draft review, content review, technical review, copy edit, final approval
- **Governance**: Amendment procedures, semantic versioning, compliance requirements

The constitution is intentionally generic to accommodate any book subject or title, which can be specified later. Templates (plan, spec, tasks) are software-focused and may need adaptation for book-writing workflows.

## Outcome

- ✅ Impact: Project constitution established for book-writing with flexible principles
- 🧪 Tests: Not applicable (documentation artifact)
- 📁 Files: Created .specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: Define book subject and title; adapt spec/plan/tasks templates for book chapters
- 🧠 Reflection: Software-focused templates need customization for book authoring workflows

## Evaluation notes (flywheel)

- Failure modes observed: None; templates designed for software may need book-specific adaptations
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Standard constitution creation flow
- Next experiment (smallest change to try): Test creating a chapter spec using current templates to identify needed adaptations
