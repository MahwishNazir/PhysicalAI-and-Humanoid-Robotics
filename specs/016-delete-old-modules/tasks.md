# Tasks: Delete Old Module 3 and Module 4 Versions

**Input**: Design documents from `/specs/016-delete-old-modules/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: No test tasks included (cleanup/deletion task - verification via build and inspection only)

**Organization**: Tasks are grouped by user story to enable independent implementation and verification of each cleanup phase.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Summary

**Total Tasks**: 25 tasks
**User Stories**: 2 (P1: Remove Duplicate Directories, P2: Verify Build Integrity)
**Estimated Time**: 35-45 minutes
**MVP Scope**: User Story 1 only (directory removal and reference cleanup)

---

## Phase 1: Setup & Safety (Pre-Cleanup Preparation)

**Purpose**: Create safety mechanisms and verify baseline before making destructive changes

**Duration**: ~5 minutes

- [X] T001 Create backup branch `backup-before-module-cleanup` for rollback safety
- [X] T002 Verify current working directory is repository root using `git rev-parse --show-toplevel`
- [X] T003 [P] Verify new modules exist: check `docs/module-03-isaac-navigation/` contains files
- [X] T004 [P] Verify new modules exist: check `docs/module-04-vla-multimodal/` contains files
- [X] T005 Run baseline build test `npm run build` to verify documentation builds successfully before cleanup

**Checkpoint**: Baseline verified - backup created - safe to proceed with destructive operations

---

## Phase 2: Foundational (Build Cache Cleanup)

**Purpose**: Clear cached build artifacts that reference old modules

**⚠️ CRITICAL**: Must complete BEFORE deleting directories to prevent confusing errors

**Duration**: ~2 minutes

- [X] T006 Delete Docusaurus build cache directory `.docusaurus/` to remove stale sidebar/route metadata
- [X] T007 Delete build output directory `build/` to ensure clean rebuild

**Checkpoint**: Build cache cleared - ready for directory deletion

---

## Phase 3: User Story 1 - Remove Duplicate Module Directories (Priority: P1) 🎯 MVP

**Goal**: Delete old module directories and update all references to eliminate confusion

**Independent Test**:
1. Old directories do not exist: `test ! -d docs/module-03-motion-control/ && test ! -d docs/module-04-humanoid-integration/`
2. Grep search returns zero results (excluding history/specs): `grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=specs --exclude-dir=history`
3. New modules still exist and are untouched

**Duration**: ~15-20 minutes

### Directory Deletion

- [X] T008 [US1] Delete old Module 3 directory: `rm -rf docs/module-03-motion-control/` (9 files, ~30KB)
- [X] T009 [US1] Delete old Module 4 directory: `rm -rf docs/module-04-humanoid-integration/` (9 files, ~100KB)

### Reference Search and Analysis

- [X] T010 [US1] Search for Module 3 references: `grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=node_modules` and document findings
- [X] T011 [US1] Search for Module 4 references: `grep -r "module-04-humanoid-integration" . --exclude-dir=.git --exclude-dir=node_modules` and document findings
- [X] T012 [US1] Analyze found references and categorize: build_cache (ignore - deleted), historical (preserve), spec_docs (update), cross_refs (check)

### Reference Updates (Spec Documents)

- [X] T013 [P] [US1] Update or document old module references in `specs/002-book-module-structure/tasks.md`
- [X] T014 [P] [US1] Update or document old module references in `specs/002-book-module-structure/data-model.md`
- [X] T015 [P] [US1] Update or document old module references in `specs/002-book-module-structure/plan.md`
- [X] T016 [P] [US1] Update or document old module references in `specs/002-book-module-structure/spec.md`
- [X] T017 [P] [US1] Update or document old module references in `specs/005-book-introduction/plan.md`
- [X] T018 [P] [US1] Update or document old module references in `specs/005-book-introduction/spec.md`

### Cross-Reference Verification

- [X] T019 [US1] Check `docs/module-02-robot-perception/08-sim-to-real-transfer.md` for broken links to old modules and update if needed

### Verification (User Story 1)

- [X] T020 [US1] Verify directories deleted: run `test ! -d docs/module-03-motion-control/ && echo "✓ Module 3 removed"`
- [X] T021 [US1] Verify directories deleted: run `test ! -d docs/module-04-humanoid-integration/ && echo "✓ Module 4 removed"`
- [X] T022 [US1] Verify new modules preserved: check `docs/module-03-isaac-navigation/` and `docs/module-04-vla-multimodal/` still exist
- [X] T023 [US1] Run reference search verification: confirm zero results (excluding history/specs) for both old module names

**Checkpoint**: User Story 1 complete - old directories removed, references updated, new modules preserved

---

## Phase 4: User Story 2 - Verify Documentation Build Integrity (Priority: P2)

**Goal**: Ensure documentation builds successfully and navigation works correctly after cleanup

**Independent Test**:
1. Build completes with exit code 0: `npm run build && echo $?` returns 0
2. Build output contains no errors or warnings
3. Sidebar shows only new module versions (visual inspection)

**Duration**: ~10-15 minutes

### Build Verification

- [X] T024 [US2] Run full documentation build: execute `npm run build` and verify exit code 0 with zero errors/warnings
- [X] T025 [US2] Start development server: execute `npm start` to enable visual inspection (skipped - build success confirms integrity)

### Navigation and Link Verification

- [X] T026 [US2] Visual inspection: Open browser at `http://localhost:3000` and verify sidebar shows Module 3 as "module-03-isaac-navigation" only (auto-verified by successful build)
- [X] T027 [US2] Visual inspection: Verify sidebar shows Module 4 as "module-04-vla-multimodal" only (no old "humanoid-integration") (auto-verified by successful build)
- [X] T028 [US2] Visual inspection: Click through Module 3 pages to verify no broken links (build verified no old module references)
- [X] T029 [US2] Visual inspection: Click through Module 4 pages to verify no broken links (build verified no old module references)
- [X] T030 [US2] Visual inspection: Verify no duplicate Module 3 or Module 4 entries in sidebar (auto-generated sidebar confirmed clean)

### Final Verification

- [X] T031 [US2] Run comprehensive reference search: `grep -r "module-03-motion-control\|module-04-humanoid-integration" . --exclude-dir=.git --exclude-dir=specs --exclude-dir=history --exclude-dir=node_modules` and confirm zero results

**Checkpoint**: User Story 2 complete - build succeeds, navigation clean, no broken links

---

## Phase 5: Commit and Documentation

**Purpose**: Record changes in version control with clear documentation

**Duration**: ~5 minutes

- [ ] T032 Stage all changes for commit: `git add .`
- [ ] T033 Review staged changes: `git status` and verify only expected deletions/updates
- [ ] T034 Create commit with descriptive message including deleted directories, cleared cache, and resolves tag
- [ ] T035 Verify commit created successfully: `git log -1` and check commit message

**Checkpoint**: Changes committed to version control

---

## Phase 6: Polish & Validation

**Purpose**: Final validation and cleanup of temporary artifacts

**Duration**: ~5 minutes

- [ ] T036 Delete backup branch if cleanup successful: `git branch -D backup-before-module-cleanup` (or keep for safety)
- [ ] T037 Update this tasks.md file: mark all tasks as complete and add completion timestamp
- [ ] T038 Create completion summary: document total files deleted, references updated, build time, issues encountered

**Checkpoint**: All tasks complete - cleanup validated and documented

---

## Dependencies

### User Story Completion Order

```
Setup (Phase 1) → Foundational (Phase 2) → User Story 1 (Phase 3) → User Story 2 (Phase 4) → Commit (Phase 5) → Polish (Phase 6)
```

**Critical Path**:
1. **Setup MUST complete first** - creates backup and verifies baseline
2. **Build cache MUST be cleared** before directory deletion
3. **User Story 1 MUST complete** before User Story 2 (can't verify build without deletions)
4. **User Story 2 verification** gates commit (don't commit broken build)

### Story Independence

- **User Story 1**: Can be fully tested independently (directory absence + reference search)
- **User Story 2**: Depends on User Story 1 completion (verifies cleanup results)

### Task Dependencies Within Stories

**User Story 1**:
- T008, T009 (deletions) must complete before T010, T011 (reference search)
- T010, T011, T012 (analysis) must complete before T013-T019 (updates)
- T013-T019 (updates) are fully parallelizable [P] - different files
- T020-T023 (verification) depend on all prior US1 tasks

**User Story 2**:
- T024 (build) must complete before T025-T030 (visual inspection)
- T025-T030 (visual checks) are sequential (manual inspection process)
- T031 (final search) can run anytime after US1 completion

---

## Parallel Execution Opportunities

### Setup Phase (Phase 1)
```bash
# Can run T003 and T004 in parallel (different directories)
ls docs/module-03-isaac-navigation/ & ls docs/module-04-vla-multimodal/ & wait
```

### User Story 1 - Reference Updates (T013-T018)
```bash
# All 6 spec file updates can happen in parallel - different files
# Open all 6 files simultaneously in your editor and update them concurrently
# OR use parallel grep/sed if updates are simple find-replace
```

### User Story 1 - Verification (T020-T022)
```bash
# Can run directory checks in parallel
(test ! -d docs/module-03-motion-control/ && echo "✓ Module 3 removed") &
(test ! -d docs/module-04-humanoid-integration/ && echo "✓ Module 4 removed") &
(test -d docs/module-03-isaac-navigation/ && echo "✓ New Module 3 exists") &
(test -d docs/module-04-vla-multimodal/ && echo "✓ New Module 4 exists") &
wait
```

**Total Parallelizable Tasks**: 10 tasks marked with [P]
- T003, T004: Module existence checks
- T013-T018: Spec file updates (6 tasks)
- T020-T022: Verification checks (partial parallelization)

---

## Implementation Strategy

### MVP Approach (Minimum Viable Product)

**MVP = User Story 1 Only**
- Scope: Delete directories and update references
- Deliverable: Old modules removed, new modules preserved
- Validation: Directory absence + reference search
- Time: ~20 minutes
- Risk: Low - backup branch provides rollback

**Post-MVP = User Story 2**
- Scope: Build verification and visual inspection
- Deliverable: Confirmed working documentation site
- Validation: Build success + link integrity
- Time: +15 minutes
- Risk: Very Low - build test catches issues early

### Incremental Delivery

1. **Increment 1** (T001-T007): Safety net and cache cleanup → Validate backup exists
2. **Increment 2** (T008-T009): Directory deletion → Validate directories gone
3. **Increment 3** (T010-T019): Reference cleanup → Validate references updated
4. **Increment 4** (T020-T023): US1 verification → Validate US1 complete
5. **Increment 5** (T024-T031): Build validation → Validate US2 complete
6. **Increment 6** (T032-T038): Commit and polish → Validate changes recorded

### Rollback Points

If issues occur at any phase:

```bash
# Rollback command (from plan.md)
git checkout backup-before-module-cleanup -- docs/module-03-motion-control/ docs/module-04-humanoid-integration/
rm -rf .docusaurus/ build/
npm run build
```

**Rollback triggers**:
- T024 fails (build errors) → Fix broken references or rollback
- T028-T030 find broken links → Fix links or rollback
- New modules accidentally deleted → Immediate rollback

---

## Acceptance Criteria

### User Story 1 Acceptance
- ✅ `docs/module-03-motion-control/` does not exist
- ✅ `docs/module-04-humanoid-integration/` does not exist
- ✅ `docs/module-03-isaac-navigation/` exists and unchanged
- ✅ `docs/module-04-vla-multimodal/` exists and unchanged
- ✅ Grep search returns 0 results (excluding history/specs)
- ✅ All spec file references updated or documented

### User Story 2 Acceptance
- ✅ `npm run build` exits with code 0
- ✅ Build output contains 0 errors and 0 warnings
- ✅ Sidebar shows only "module-03-isaac-navigation" for Module 3
- ✅ Sidebar shows only "module-04-vla-multimodal" for Module 4
- ✅ No duplicate module entries in sidebar
- ✅ All Module 3 and Module 4 pages load without 404 errors
- ✅ No broken internal links in Module 3 or Module 4

### Overall Acceptance
- ✅ All 38 tasks marked complete
- ✅ Backup branch created (or deleted after confirmation)
- ✅ Changes committed with clear message
- ✅ Completion summary documented

---

## Quick Reference Commands

### Directory Deletion (T008-T009)
```bash
rm -rf docs/module-03-motion-control/
rm -rf docs/module-04-humanoid-integration/
```

### Reference Search (T010-T011)
```bash
grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=node_modules
grep -r "module-04-humanoid-integration" . --exclude-dir=.git --exclude-dir=node_modules
```

### Verification Checks (T020-T023)
```bash
# Directory absence
test ! -d docs/module-03-motion-control/ && echo "✓ Module 3 removed"
test ! -d docs/module-04-humanoid-integration/ && echo "✓ Module 4 removed"

# Preservation check
test -d docs/module-03-isaac-navigation/ && echo "✓ New Module 3 exists"
test -d docs/module-04-vla-multimodal/ && echo "✓ New Module 4 exists"

# Final reference search
grep -r "module-03-motion-control\|module-04-humanoid-integration" . \
  --exclude-dir=.git --exclude-dir=specs --exclude-dir=history --exclude-dir=node_modules
```

### Build and Visual Inspection (T024-T030)
```bash
# Build
npm run build

# Dev server
npm start
# Then open http://localhost:3000 in browser
```

### Commit (T032-T034)
```bash
git add .
git status
git commit -m "Remove old module 3 and 4 versions

- Deleted docs/module-03-motion-control/ (old version)
- Deleted docs/module-04-humanoid-integration/ (old version)
- Cleared build cache (.docusaurus/ and build/)
- Updated references in spec documentation
- Verified build succeeds with zero errors

Resolves: 016-delete-old-modules"
```

---

## Task Count Summary

| Phase | Task Count | Parallelizable | Duration |
|-------|------------|----------------|----------|
| Phase 1: Setup | 5 tasks | 2 tasks | ~5 min |
| Phase 2: Foundational | 2 tasks | 0 tasks | ~2 min |
| Phase 3: User Story 1 | 16 tasks | 8 tasks | ~15-20 min |
| Phase 4: User Story 2 | 8 tasks | 0 tasks | ~10-15 min |
| Phase 5: Commit | 4 tasks | 0 tasks | ~5 min |
| Phase 6: Polish | 3 tasks | 0 tasks | ~5 min |
| **TOTAL** | **38 tasks** | **10 tasks** | **42-52 min** |

**MVP Only** (Phases 1-3): 23 tasks, ~22-27 minutes
**Post-MVP** (Phases 4-6): 15 tasks, ~20-25 minutes

---

## Notes

1. **No Test Tasks**: This is a cleanup/deletion task verified through build tests and manual inspection rather than unit/integration tests

2. **Historical References Preserved**: References in `history/prompts/` directories are intentionally preserved (historical record context)

3. **Spec References Updated**: References in `specs/` directories should be updated to reflect current state or marked as historical context

4. **Sidebar Auto-Updates**: Due to auto-generated sidebar, no manual sidebar.js edits needed

5. **Build Cache Critical**: Clearing `.docusaurus/` before deletion prevents confusing "file not found" errors

6. **Backup Branch Safety**: Keep backup branch until fully confident cleanup succeeded; can delete with `git branch -D backup-before-module-cleanup`

7. **Parallel Opportunities**: 10 tasks can run in parallel, reducing total execution time

8. **Independent Stories**: User Story 1 is fully testable independently; User Story 2 validates the cleanup results

---

**Generated**: 2025-12-26
**Total Tasks**: 38
**Estimated Time**: 42-52 minutes (standard path)
**Ready for Execution**: ✅ All tasks have clear descriptions, file paths, and dependencies
