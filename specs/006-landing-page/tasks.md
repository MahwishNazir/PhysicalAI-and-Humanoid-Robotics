# Tasks: Landing Page for Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/006-landing-page/`
**Prerequisites**: plan.md (complete), spec.md (4 user stories), research.md (Docusaurus routing), quickstart.md (7-phase guide)

**Tests**: No automated tests requested in specification. Manual testing via Lighthouse audits and user flow validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus Static Site**: `src/pages/`, `static/img/`, `src/css/`, `docusaurus.config.js`
- All paths relative to repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Docusaurus configuration and directory structure

- [x] T001 Verify Docusaurus is installed and `npm start` runs successfully
- [x] T002 [P] Create directory structure: `src/pages/`, `src/pages/components/Hero/`, `src/pages/components/Footer/`
- [x] T003 [P] Update `docusaurus.config.js` to change docs `routeBasePath` from `/` to `/intro`
- [ ] T004 [P] Verify docs are accessible at `/intro` after route change (test: `npm start` and navigate to `http://localhost:3000/intro`)

**Checkpoint**: Docusaurus routing configured - landing page can now be created at `/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core theme configuration and CSS setup that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 [P] Update `src/css/custom.css` with Infima color variable overrides for robotics/AI theme (primary: #303F9F, secondary: #00ACC1)
- [x] T006 [P] Add focus indicator styles to `src/css/custom.css` for keyboard accessibility (`*:focus-visible { outline: 2px solid var(--ifm-color-primary); }`)
- [ ] T007 Verify light/dark mode toggle works in navbar (test: click sun/moon icon, verify colors change)

**Checkpoint**: Theme foundation ready - user story components can now use Infima variables

---

## Phase 3: User Story 1 - First-Time Visitor Discovery (Priority: P1) 🎯 MVP

**Goal**: Prospective learner visits root URL, sees book title/tagline/hero image, clicks "Start Reading" to navigate to `/intro`

**Independent Test**: Navigate to `http://localhost:3000/`, verify title/tagline/"Start Reading" button visible, click button and reach `/intro` with Docusaurus sidebar

### Implementation for User Story 1

- [x] T008 [US1] Create `src/pages/index.tsx` with basic structure (imports Layout, exports LandingPage function)
- [x] T009 [US1] Create `src/pages/index.module.css` with flexbox centering for main content area
- [x] T010 [P] [US1] Create `src/pages/components/Hero/Hero.tsx` with hero section structure (title, tagline, "Start Reading" button, hero image placeholder)
- [x] T011 [P] [US1] Create `src/pages/components/Hero/Hero.module.css` with responsive styles (mobile-first: column layout, desktop 996px+: row layout)
- [x] T012 [US1] Import Hero component into `src/pages/index.tsx` and render in main section
- [x] T013 [US1] Add book title "Physical AI & Humanoid Robotics" as H1 in `Hero.tsx`
- [x] T014 [US1] Add tagline "Master robot middleware, simulation, AI perception, and voice-controlled autonomy" in `Hero.tsx`
- [x] T015 [US1] Add "Start Reading" button using Docusaurus Link component with `href="/intro"` and Infima classes (`button button--primary button--lg`)
- [x] T016 [P] [US1] Add hero image placeholder to `static/img/hero-robot.svg` (or download royalty-free robot image from Unsplash/Pexels as hero-robot.webp, <100KB)
- [x] T017 [US1] Add `<img>` tag to `Hero.tsx` with `src="/img/hero-robot.webp"` (or .svg), alt text "Futuristic humanoid robot illustration representing Physical AI", `loading="lazy"`, `decoding="async"`
- [ ] T018 [US1] Test navigation: Click "Start Reading" button, verify navigation to `/intro` with Docusaurus sidebar visible
- [ ] T019 [US1] Test mobile responsive layout at 375px width (Chrome DevTools → Toggle device toolbar): Verify content readable, button tappable, no horizontal scroll
- [ ] T020 [US1] Test desktop layout at 996px+ width: Verify hero text and image side-by-side
- [ ] T021 [US1] Test keyboard navigation: Press Tab key, verify "Start Reading" button is focusable and has visible outline
- [ ] T022 [US1] Run Lighthouse performance audit (production build: `npm run build && npm run serve`): Target desktop score >90

**Checkpoint**: User Story 1 COMPLETE - MVP functional (landing page with "Start Reading" CTA works)

---

## Phase 4: User Story 2 - Contact & Repository Access (Priority: P2)

**Goal**: Visitor scrolls to footer, clicks GitHub repository link (opens new tab) or email contact (opens mail client)

**Independent Test**: Scroll to bottom of landing page, verify GitHub and email links present, click each and verify GitHub opens in new tab, email opens mail client

### Implementation for User Story 2

- [x] T023 [P] [US2] Create `src/pages/components/Footer/Footer.tsx` with footer structure (GitHub link, email link, copyright)
- [x] T024 [P] [US2] Create `src/pages/components/Footer/Footer.module.css` with responsive styles (mobile: stacked column, desktop 996px+: horizontal row)
- [x] T025 [US2] Import Footer component into `src/pages/index.tsx` and render after main section
- [x] T026 [US2] Add GitHub repository link to `Footer.tsx` with `href="https://github.com/username/physical-ai-book"` (TODO: update with real URL), `target="_blank"`, `rel="noopener noreferrer"`, label "📦 GitHub Repository"
- [x] T027 [US2] Add email contact link to `Footer.tsx` with `href="mailto:contact@example.com"` (TODO: update with real email), label "✉️ Contact"
- [x] T028 [US2] Add copyright text to `Footer.tsx`: `© {currentYear} Your Name. Built with Docusaurus.` (use `new Date().getFullYear()` for dynamic year)
- [ ] T029 [US2] Test GitHub link: Click link, verify opens in new tab
- [ ] T030 [US2] Test email link: Click link, verify opens default email client with address pre-filled
- [ ] T031 [US2] Test footer responsive layout at 375px width: Verify links are stacked vertically, easily tappable, no horizontal scroll
- [ ] T032 [US2] Test footer responsive layout at 996px+ width: Verify links are horizontally aligned

**Checkpoint**: User Story 2 COMPLETE - Footer with GitHub and email contact functional

---

## Phase 5: User Story 3 - Authentication Awareness (Priority: P3)

**Goal**: Visitor sees "Sign In" and "Sign Up" buttons in hero section; clicking them shows placeholder message "User accounts coming soon!"

**Independent Test**: Verify "Sign In" and "Sign Up" buttons visible, click each and verify placeholder message appears and auto-hides after 3 seconds

### Implementation for User Story 3

- [x] T033 [US3] Add React state to `Hero.tsx` using `useState` hook for auth message visibility (`const [showAuthMessage, setShowAuthMessage] = useState(false)`)
- [x] T034 [US3] Add `handleAuthClick` function to `Hero.tsx` that sets `showAuthMessage` to true and uses `setTimeout` to hide after 3 seconds
- [x] T035 [P] [US3] Add "Sign In" button to `Hero.tsx` with Infima classes (`button button--secondary button--outline`), `onClick={handleAuthClick}`
- [x] T036 [P] [US3] Add "Sign Up" button to `Hero.tsx` with Infima classes (`button button--secondary button--outline`), `onClick={handleAuthClick}`
- [x] T037 [US3] Add auth message div to `Hero.tsx` that conditionally renders when `showAuthMessage` is true, with text "User accounts coming soon! For now, click 'Start Reading' to explore the book."
- [x] T038 [US3] Style auth message div in `Hero.module.css` with Infima warning color (`background: var(--ifm-color-warning-lightest)`, `border: 1px solid var(--ifm-color-warning)`)
- [ ] T039 [US3] Test auth buttons: Click "Sign In", verify placeholder message appears
- [ ] T040 [US3] Test auth message auto-hide: Wait 3 seconds after clicking auth button, verify message disappears
- [ ] T041 [US3] Verify "Start Reading" button is visually more prominent than auth buttons (larger size with `button--lg`, primary color vs secondary/outline)

**Checkpoint**: User Story 3 COMPLETE - Auth placeholder UI ready for future Better Auth integration

---

## Phase 6: User Story 4 - Professional Visual Design (Priority: P3)

**Goal**: Landing page has professional robotics/AI theme with custom colors, readable typography, responsive layout 320px-1920px

**Independent Test**: Visual review (screenshot testing) and Lighthouse accessibility audit (score 100, WCAG AA compliance)

### Implementation for User Story 4

- [x] T042 [P] [US4] Verify hero image is robotics/AI themed (robot silhouette, mechanical arm, or futuristic humanoid) - if placeholder SVG, replace with real image from Unsplash/Pexels
- [x] T043 [P] [US4] Optimize hero image to <100KB file size using Squoosh.app or TinyPNG, convert to WebP format
- [x] T044 [US4] Verify light mode colors in `src/css/custom.css`: Primary Indigo 700 (#303F9F), Secondary Cyan 600 (#00ACC1), verify contrast ratios meet WCAG AA (4.5:1 for text)
- [x] T045 [US4] Verify dark mode colors in `src/css/custom.css`: `[data-theme='dark']` selector has lighter primary (#5C6BC0), dark background (#121212)
- [ ] T046 [US4] Toggle dark mode in navbar, verify all colors invert appropriately, hero image remains visible
- [x] T047 [US4] Verify typography: Body text minimum 16px (`--ifm-font-size-base`), system font stack for performance
- [ ] T048 [US4] Test responsive layout at 320px width (iPhone SE): Verify layout usable, no horizontal scroll, buttons full width on mobile
- [ ] T049 [US4] Test responsive layout at 768px width (iPad): Verify content stacks vertically, buttons inline
- [ ] T050 [US4] Test responsive layout at 996px width (Desktop): Verify hero content side-by-side (text left, image right)
- [ ] T051 [US4] Test responsive layout at 1920px width (Large Desktop): Verify content centered with max-width, not full screen
- [ ] T052 [US4] Run Lighthouse accessibility audit (production build): Target score 100, verify zero WCAG AA violations, proper heading hierarchy (h1 > h2), semantic HTML

**Checkpoint**: User Story 4 COMPLETE - Professional visual design with accessibility compliance

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final quality checks, performance optimization, and deployment readiness

- [x] T053 [P] Update placeholder GitHub URL in `Footer.tsx` from `https://github.com/username/physical-ai-book` to real repository URL (if available)
- [x] T054 [P] Update placeholder email in `Footer.tsx` from `contact@example.com` to real contact email (if available)
- [x] T055 [P] Update placeholder author name in `Footer.tsx` from "Your Name" to real author name
- [x] T056 [P] Add "Home" link to navbar in `docusaurus.config.js` (optional): `{ to: '/', label: 'Home', position: 'left' }`
- [ ] T057 Test full user flow (all 4 user stories sequentially): Navigate to `/`, verify title/tagline/image/buttons, click "Start Reading" → `/intro`, scroll to footer, click GitHub (new tab), click email (mail client), click auth buttons (placeholder message)
- [ ] T058 Test keyboard navigation full flow: Tab through all interactive elements (Start Reading, Sign In, Sign Up, GitHub, Email), verify all have visible focus indicators
- [ ] T059 Run final Lighthouse audit (production build): Verify Performance >90 desktop, >75 mobile; Accessibility 100; Best Practices >90
- [x] T060 Verify semantic HTML with HTML validator: Proper heading hierarchy (h1 → h2, no skipped levels), `<section>`, `<footer>` tags used appropriately
- [ ] T061 Cross-browser testing: Test in Chrome, Firefox, Safari, Edge (last 2 versions), verify consistent appearance and functionality
- [x] T062 Create documentation: Add comment block to `src/pages/index.tsx` explaining landing page structure and how to update content
- [x] T063 [P] Create backup of original `docs/intro.md` if not already backed up (verify book introduction content preserved during routing change)
- [x] T064 Final verification: Ensure all TODO comments in code are resolved or documented in project tracking

**Checkpoint**: Landing page feature COMPLETE - Ready for deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T003-T004: routing must be configured) - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion (T005-T007: theme variables must be configured)
  - User Story 1 (P1 - MVP): Can start after Foundational
  - User Story 2 (P2): Can start after Foundational, independent of US1 (Footer is separate component)
  - User Story 3 (P3): Depends on US1 (adds auth buttons to Hero component created in US1)
  - User Story 4 (P3): Can start after Foundational, but should validate after US1-US3 complete
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Independent after Foundational - Creates Hero component and landing page entry
- **User Story 2 (P2)**: Independent after Foundational - Creates Footer component
- **User Story 3 (P3)**: Depends on User Story 1 (modifies Hero component with auth buttons)
- **User Story 4 (P3)**: Independent after Foundational - Visual design and optimization

**Dependency Graph**:
```
Setup (Phase 1)
    ↓
Foundational (Phase 2)
    ↓
    ├─→ User Story 1 (P1) ─→ User Story 3 (P3)
    ├─→ User Story 2 (P2)
    └─→ User Story 4 (P3)
         ↓
    Polish (Phase 7)
```

### Within Each User Story

**User Story 1** (Hero Component):
- T008-T009 (index.tsx, CSS) before T010-T011 (Hero component)
- T010-T011 (Hero structure) before T012-T017 (Hero content)
- T016 (hero image) can run in parallel with T010-T015 (React components)
- T018-T022 (testing) must run after T008-T017 (implementation)

**User Story 2** (Footer Component):
- T023-T024 (Footer structure) can run in parallel
- T025-T028 (Footer implementation) sequential (imports → content)
- T029-T032 (testing) must run after T023-T028 (implementation)

**User Story 3** (Auth Placeholders):
- T033-T034 (state management) before T035-T037 (buttons and message)
- T035-T036 (buttons) can run in parallel
- T038-T041 (styling and testing) must run after T033-T037 (implementation)

**User Story 4** (Visual Design):
- T042-T043 (image optimization) can run in parallel with T044-T047 (theme verification)
- T048-T051 (responsive testing) can run in parallel (different viewports)
- T052 (accessibility audit) must run last (validates all visual design)

### Parallel Opportunities

**Setup Phase** (3 tasks can run in parallel):
- T002 (directory creation)
- T003 (config update)
- T004 (routing verification) depends on T003

**Foundational Phase** (all 3 tasks can run in parallel):
- T005 (color variables)
- T006 (focus indicators)
- T007 (theme toggle test) depends on T005

**User Story 1** (5 parallel opportunities):
- T008-T009 sequential
- T010-T011 can run in parallel (Hero component structure)
- T016 can run in parallel with T010-T015 (image while building component)
- T019-T020 can run in parallel (mobile and desktop testing)

**User Story 2** (2 parallel opportunities):
- T023-T024 can run in parallel (Footer component structure)
- T029-T030 can run in parallel (GitHub and email link testing)
- T031-T032 can run in parallel (mobile and desktop footer testing)

**User Story 3** (2 parallel opportunities):
- T035-T036 can run in parallel (Sign In and Sign Up buttons)
- T039-T040 can run in parallel (button click testing)

**User Story 4** (8 parallel opportunities):
- T042-T043 can run in parallel (image replacement and optimization)
- T044-T045 can run in parallel (light and dark mode theme verification)
- T048-T051 can run in parallel (all 4 responsive viewport tests)

**Polish Phase** (5 parallel opportunities):
- T053-T055 can run in parallel (placeholder updates)
- T057-T061 sequential (testing after implementation)
- T063 can run in parallel with other tasks

**Total Parallelizable Tasks**: 25 out of 64 tasks (39%)

---

## Parallel Example: User Story 1

```bash
# Launch hero image optimization while building Hero component:
Task: "Add hero image placeholder to static/img/hero-robot.svg"
Task: "Create src/pages/components/Hero/Hero.tsx with hero section structure"

# Launch mobile and desktop testing in parallel:
Task: "Test mobile responsive layout at 375px width"
Task: "Test desktop layout at 996px+ width"
```

---

## Parallel Example: User Story 4

```bash
# Launch all responsive viewport tests together:
Task: "Test responsive layout at 320px width (iPhone SE)"
Task: "Test responsive layout at 768px width (iPad)"
Task: "Test responsive layout at 996px width (Desktop)"
Task: "Test responsive layout at 1920px width (Large Desktop)"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004) → Docusaurus routing configured
2. Complete Phase 2: Foundational (T005-T007) → Theme variables ready
3. Complete Phase 3: User Story 1 (T008-T022) → Landing page with "Start Reading" functional
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Navigate to `/`, verify title/tagline/hero image/"Start Reading" button
   - Click button, verify navigation to `/intro` with sidebar
   - Test mobile responsive (375px), desktop (996px), keyboard navigation
   - Run Lighthouse audit (performance >90)
5. **MVP COMPLETE** - Deploy/demo if ready

**Estimated Time for MVP**: 4.5 hours (per quickstart.md Phase 1-3)

---

### Incremental Delivery

1. **Foundation** (Setup + Foundational): T001-T007 → 30 minutes
2. **MVP** (User Story 1): T008-T022 → 1.5 hours → Test independently → Deploy/Demo
3. **Add Footer** (User Story 2): T023-T032 → 45 minutes → Test independently → Deploy/Demo
4. **Add Auth Placeholders** (User Story 3): T033-T041 → 30 minutes → Test independently → Deploy/Demo
5. **Visual Polish** (User Story 4): T042-T052 → 1 hour → Test independently → Deploy/Demo
6. **Final Polish** (Phase 7): T053-T064 → 30 minutes → Deploy/Demo

**Total Estimated Time**: 4.5 hours MVP, 6-7 hours all user stories (matches quickstart.md estimate)

Each increment adds value without breaking previous stories.

---

### Parallel Team Strategy

With multiple developers:

1. **All together**: Complete Setup + Foundational (T001-T007)
2. **Once Foundational is done**:
   - **Developer A**: User Story 1 (T008-T022) - Hero component
   - **Developer B**: User Story 2 (T023-T032) - Footer component
   - **Developer C**: User Story 4 (T042-T052) - Visual design and image optimization
3. **After US1 complete**:
   - **Developer A**: User Story 3 (T033-T041) - Auth placeholders (depends on US1 Hero component)
4. **All together**: Polish (T053-T064) - Final integration testing

**Team Efficiency**: User Stories 1, 2, and 4 can be developed in parallel after Foundational phase.

---

## Notes

- **[P] tasks** = Different files, no dependencies, safe to parallelize
- **[Story] label** = Maps task to specific user story for traceability
- **Each user story independently testable**: US1 (landing page works), US2 (footer works), US3 (auth placeholders work), US4 (visual design meets standards)
- **No automated tests requested**: Specification requires manual testing via Lighthouse audits, user flow validation, and responsive design testing
- **Commit strategy**: Commit after each logical group (e.g., after Hero component complete, after Footer complete)
- **Stop at any checkpoint** to validate story independently before proceeding
- **Avoid**: Cross-story dependencies that break independence (US3 depends on US1 Hero component, but US2 and US4 are fully independent)
- **File conflicts**: US3 modifies Hero component created in US1, so US3 must run after US1. US2 (Footer) and US4 (Visual) are independent.

---

## Task Count Summary

- **Setup**: 4 tasks (T001-T004)
- **Foundational**: 3 tasks (T005-T007)
- **User Story 1 (P1 - MVP)**: 15 tasks (T008-T022)
- **User Story 2 (P2)**: 10 tasks (T023-T032)
- **User Story 3 (P3)**: 9 tasks (T033-T041)
- **User Story 4 (P3)**: 11 tasks (T042-T052)
- **Polish**: 12 tasks (T053-T064)

**Total Tasks**: 64

**Parallelizable Tasks**: 25 (39%)

**MVP Tasks (Setup + Foundational + US1)**: 22 tasks

**Format Validation**: ✅ All 64 tasks follow checklist format (checkbox, ID, [P] and [Story] labels where applicable, file paths)
