---
id: 003
title: landing-page-tasks
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 006-landing-page
branch: 006-landing-page
user: User
command: /sp.tasks
labels: ["tasks", "task-generation", "landing-page", "react", "docusaurus"]
links:
  spec: specs/006-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-landing-page/tasks.md
tests:
 - Format validation: All 64 tasks follow checklist format (checkbox, ID, [P], [Story], file paths)
 - Independent test criteria defined for all 4 user stories
 - Parallel opportunities identified (25/64 tasks = 39%)
---

## Prompt

```
/sp.tasks
```

(No additional arguments provided)

## Response snapshot

Generated comprehensive task breakdown (64 tasks) organized by 4 user stories from spec.md, adapted for Docusaurus landing page implementation:

### Task Organization

**7 Phases Total**:
1. **Setup** (4 tasks): Docusaurus verification, directory creation, routing configuration
2. **Foundational** (3 tasks): Theme variables, focus indicators, light/dark mode verification
3. **User Story 1 - First-Time Visitor Discovery [P1, MVP]** (15 tasks): Landing page entry, Hero component, "Start Reading" CTA
4. **User Story 2 - Contact & Repository Access [P2]** (10 tasks): Footer component with GitHub and email links
5. **User Story 3 - Authentication Awareness [P3]** (9 tasks): Auth placeholder buttons with message
6. **User Story 4 - Professional Visual Design [P3]** (11 tasks): Hero image, theme verification, responsive testing, accessibility audit
7. **Polish** (12 tasks): Placeholder updates, full user flow testing, cross-browser validation

### User Story Breakdown

**User Story 1 (P1 - MVP)**: 15 tasks
- **Goal**: Landing page at `/` with title, tagline, hero image, "Start Reading" button → navigates to `/intro`
- **Independent Test**: Navigate to root URL, click "Start Reading", verify navigation to `/intro` with Docusaurus sidebar
- **Tasks**: Create `index.tsx`, Hero component, CSS modules, "Start Reading" button, hero image, responsive testing, keyboard navigation, Lighthouse audit
- **Parallel Opportunities**: T010-T011 (Hero component structure), T016 (image) parallel with T010-T015 (component development), T019-T020 (mobile/desktop testing)

**User Story 2 (P2)**: 10 tasks
- **Goal**: Footer with GitHub repository link and email contact
- **Independent Test**: Scroll to footer, verify GitHub link opens new tab, email opens mail client
- **Tasks**: Create Footer component, CSS module, GitHub link (with `target="_blank" rel="noopener noreferrer"`), email `mailto:` link, copyright, responsive testing
- **Parallel Opportunities**: T023-T024 (Footer structure), T029-T030 (link testing), T031-T032 (responsive testing)

**User Story 3 (P3)**: 9 tasks
- **Goal**: "Sign In" and "Sign Up" buttons in hero with placeholder message "User accounts coming soon!"
- **Independent Test**: Click auth buttons, verify placeholder message appears and auto-hides after 3 seconds
- **Tasks**: Add React `useState` for message visibility, `handleAuthClick` function, auth buttons with Infima classes, conditional message rendering, styling, testing
- **Dependencies**: Depends on US1 (modifies Hero component)
- **Parallel Opportunities**: T035-T036 (Sign In/Sign Up buttons), T039-T040 (click testing)

**User Story 4 (P3)**: 11 tasks
- **Goal**: Professional robotics/AI theme, responsive 320px-1920px, WCAG AA accessibility
- **Independent Test**: Visual review (screenshot testing), Lighthouse accessibility audit (score 100)
- **Tasks**: Verify/optimize hero image (<100KB WebP), verify light/dark mode colors, responsive testing at 5 viewports, accessibility audit
- **Parallel Opportunities**: T042-T043 (image optimization), T044-T045 (theme verification), T048-T051 (all 4 responsive viewport tests)

### Task Format Compliance

**All 64 tasks follow required checklist format**:
- ✅ Checkbox (`- [ ]`) at start
- ✅ Sequential Task ID (T001-T064)
- ✅ [P] marker for parallelizable tasks (25 tasks marked)
- ✅ [Story] label for user story tasks (US1, US2, US3, US4)
- ✅ Clear description with file path (e.g., `src/pages/index.tsx`, `static/img/hero-robot.webp`)

**Examples**:
- `- [ ] T001 Verify Docusaurus is installed and npm start runs successfully`
- `- [ ] T010 [P] [US1] Create src/pages/components/Hero/Hero.tsx with hero section structure`
- `- [ ] T023 [P] [US2] Create src/pages/components/Footer/Footer.tsx with footer structure`
- `- [ ] T042 [P] [US4] Verify hero image is robotics/AI themed (robot silhouette, mechanical arm, or futuristic humanoid)`

### Parallel Execution Opportunities

**39% of tasks (25/64) can run in parallel**:

**Setup**: 2 tasks (T002 directory creation, T003 config update can run together)

**Foundational**: 2 tasks (T005 color variables, T006 focus indicators)

**User Story 1**: 5 parallel opportunities (T010-T011 Hero structure, T016 image, T019-T020 responsive testing)

**User Story 2**: 4 parallel opportunities (T023-T024 Footer structure, T029-T030 link testing, T031-T032 responsive testing)

**User Story 3**: 2 parallel opportunities (T035-T036 auth buttons, T039-T040 testing)

**User Story 4**: 8 parallel opportunities (T042-T043 image, T044-T045 theme, T048-T051 all 4 viewports)

**Polish**: 4 parallel opportunities (T053-T055 placeholder updates, T063)

**Example parallel launch for US4 (responsive testing)**:
```bash
Task: "Test responsive layout at 320px width (iPhone SE)"
Task: "Test responsive layout at 768px width (iPad)"
Task: "Test responsive layout at 996px width (Desktop)"
Task: "Test responsive layout at 1920px width (Large Desktop)"
```

### Implementation Strategy

**MVP First** (User Story 1 only):
1. Setup + Foundational (7 tasks)
2. User Story 1 - Landing page with "Start Reading" (15 tasks)
3. Validate independently: Navigate to `/`, click button, verify navigation to `/intro`
4. **MVP Complete**: 22 tasks total, 4.5 hours estimated

**Incremental Delivery**:
1. MVP (US1) → Test independently ✅
2. Add US2 (Footer) → Test independently ✅
3. Add US3 (Auth placeholders) → Test independently ✅
4. Add US4 (Visual design) → Test independently ✅
5. Polish → Final QA ✅

**Total Estimated Time**: 6-7 hours for all user stories (matches quickstart.md estimate)

### Dependencies

**Phase Dependencies**:
- Setup (Phase 1) → Foundational (Phase 2) → User Stories (Phase 3-6) → Polish (Phase 7)
- Foundational BLOCKS all user story work (must configure theme variables first)

**User Story Dependencies**:
- US1 (P1): Independent after Foundational
- US2 (P2): Independent after Foundational (Footer is separate component)
- US3 (P3): Depends on US1 (modifies Hero component)
- US4 (P3): Independent after Foundational

**Dependency Graph**:
```
Setup → Foundational
          ↓
    ├─→ US1 ─→ US3
    ├─→ US2
    └─→ US4
         ↓
      Polish
```

**Parallel Team Strategy**: With 3 developers, US1, US2, and US4 can run in parallel after Foundational. US3 starts after US1 complete.

### Success Criteria Mapping

Tasks directly map to spec.md success criteria:
- **SC-003** (Desktop load <3s, Lighthouse >90): T022 (Lighthouse performance audit)
- **SC-004** (Mobile 3G load <5s, Lighthouse >75): T022 (mobile performance)
- **SC-005** (Zero WCAG AA violations): T052 (Lighthouse accessibility audit)
- **SC-006** (Usable 320px-1920px): T048-T051 (responsive testing at 4 viewports)
- **SC-007** (100% navigation success to /intro): T018 (navigation testing)
- **SC-008** (Footer links 100% click success): T029-T030 (link testing)
- **SC-009** (Semantic HTML with heading hierarchy): T060 (HTML validation)

### Validation Tasks

**No automated tests** - specification requires manual testing:
- T018 (navigation testing)
- T019-T020 (responsive testing - mobile and desktop)
- T021 (keyboard navigation)
- T022 (Lighthouse performance audit)
- T029-T032 (footer link testing and responsive)
- T039-T041 (auth placeholder testing)
- T046 (dark mode verification)
- T048-T051 (responsive testing at 4 viewports)
- T052 (Lighthouse accessibility audit)
- T057-T061 (full user flow, keyboard navigation, Lighthouse, HTML validation, cross-browser)

**All validation manual** via Lighthouse audits, user flow testing, responsive design testing (no automated unit/integration tests requested)

---

## Outcome

- ✅ Impact: Generated 64 actionable tasks organized by 4 prioritized user stories; MVP defined (US1 = 15 tasks + 7 setup/foundational = 22 tasks total); 39% parallelizable
- 🧪 Tests: Format validation PASS (all tasks follow checklist format); Independent test criteria defined for each user story; No automated tests (manual testing via Lighthouse, user flow validation)
- 📁 Files: specs/006-landing-page/tasks.md (complete task list with dependencies, parallel opportunities, implementation strategies)
- 🔁 Next prompts: `/sp.implement` to execute tasks and build landing page
- 🧠 Reflection: Successfully adapted software development task template for Docusaurus landing page implementation. Key adaptations: (1) No automated tests - manual validation via Lighthouse and user flow testing, (2) Parallel opportunities based on independent components (Hero, Footer) and testing viewports, (3) MVP defined as minimum UI for first-time visitor (Hero with "Start Reading" CTA), (4) Dependencies based on React component structure (US3 modifies US1 Hero component).

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation completed successfully with all validations passing
- Graders run and results (PASS/FAIL): Format validation PASS (64/64 tasks compliant), Independent test criteria PASS (all 4 stories have clear validation), Parallel opportunities PASS (25/64 = 39%)
- Prompt variant (if applicable): Adapted software task template for UI-only feature (no backend, no automated tests)
- Next experiment (smallest change to try): Consider adding explicit "screenshot testing" task for visual regression (capture screenshots at each viewport, compare across theme changes) to automate visual QA
