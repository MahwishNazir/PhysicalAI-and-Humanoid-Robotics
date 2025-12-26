# Data Model: Delete Old Module 3 and Module 4 Versions

**Feature**: 016-delete-old-modules
**Date**: 2025-12-26
**Phase**: 1 - Design

## Overview

This is a **cleanup/deletion task** rather than a feature with persistent data. However, we can model the "entities" involved in the cleanup process to ensure systematic and complete removal.

## Entities

### 1. Module Directory

Represents a documentation module directory that needs to be removed.

**Attributes**:
- `path`: Absolute or relative path to the module directory
- `name`: Human-readable module name (e.g., "Motion Control", "Humanoid Integration")
- `version`: Version identifier (e.g., "old", "new")
- `status`: Current status (exists, deleted, error)
- `file_count`: Number of files within the directory
- `total_size`: Total size of all files in bytes

**Instances**:
1. Old Module 3:
   - `path`: `docs/module-03-motion-control/`
   - `name`: "Module 3: Motion Control (Old Version)"
   - `version`: "old"
   - `status`: "exists" → "deleted"
   - `file_count`: 9 (8 markdown files + 1 _category_.json)
   - `total_size`: ~30KB

2. Old Module 4:
   - `path`: `docs/module-04-humanoid-integration/`
   - `name`: "Module 4: Humanoid Integration (Old Version)"
   - `version`: "old"
   - `status`: "exists" → "deleted"
   - `file_count`: 9 (8 markdown files + 1 _category_.json)
   - `total_size`: ~100KB

**State Transitions**:
```
exists → marked_for_deletion → deleted → verified_removed
                              ↓
                           error → rollback
```

---

### 2. Reference Location

Represents a file or location that references the old module directories.

**Attributes**:
- `file_path`: Path to the file containing the reference
- `reference_type`: Type of reference (sidebar, build_cache, cross_reference, spec_doc)
- `line_number`: Line number where reference appears (if applicable)
- `reference_text`: Actual text containing the reference
- `action_required`: Action needed (delete, update, preserve)
- `status`: Current status (pending, fixed, verified)

**Categories**:

1. **Build Cache References** (action: delete)
   - `file_path`: `.docusaurus/**/*.json`
   - `reference_type`: "build_cache"
   - `action_required`: "delete_cache_directory"

2. **Spec Document References** (action: update or preserve)
   - `file_path`: `specs/002-book-module-structure/tasks.md`
   - `reference_type`: "spec_doc"
   - `action_required`: "update"

   - `file_path`: `history/prompts/*/` (various)
   - `reference_type`: "historical_record"
   - `action_required`: "preserve"

3. **Cross-Module References** (action: verify and update if broken)
   - `file_path`: `docs/module-02-robot-perception/08-sim-to-real-transfer.md`
   - `reference_type`: "cross_reference"
   - `action_required`: "verify_and_update"

**State Transitions**:
```
pending → identified → fixed → verified
                     ↓
                   skipped (for historical records)
```

---

### 3. Verification Check

Represents a verification step in the cleanup process.

**Attributes**:
- `check_name`: Name of the verification check
- `check_type`: Type (directory_absent, reference_search, build_success, link_check, visual_inspection)
- `expected_result`: What should happen if successful
- `actual_result`: What actually happened
- `status`: Pass/Fail/Pending
- `error_message`: Error details if failed

**Instances**:

1. Directory Absence Check:
   - `check_name`: "Old Module 3 Directory Removed"
   - `check_type`: "directory_absent"
   - `expected_result`: "Directory does not exist"
   - `command`: `test ! -d docs/module-03-motion-control/`

2. Directory Absence Check:
   - `check_name`: "Old Module 4 Directory Removed"
   - `check_type`: "directory_absent"
   - `expected_result`: "Directory does not exist"
   - `command`: `test ! -d docs/module-04-humanoid-integration/`

3. Reference Search Check:
   - `check_name`: "No References to module-03-motion-control"
   - `check_type`: "reference_search"
   - `expected_result`: "Zero matches (excluding git history and specs)"
   - `command`: `grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=specs`

4. Build Success Check:
   - `check_name`: "Documentation Build Success"
   - `check_type`: "build_success"
   - `expected_result`: "Build completes with exit code 0, no errors"
   - `command`: `npm run build`

5. Link Check:
   - `check_name`: "No Broken Links"
   - `check_type`: "link_check"
   - `expected_result`: "Zero broken links in built site"

6. Sidebar Inspection:
   - `check_name`: "Clean Sidebar Navigation"
   - `check_type`: "visual_inspection"
   - `expected_result`: "Only new module versions visible, no duplicates"

---

### 4. Cleanup Operation

Represents a single cleanup operation in the execution sequence.

**Attributes**:
- `operation_id`: Unique identifier (sequential number)
- `operation_name`: Human-readable name
- `operation_type`: Type (backup, delete_directory, delete_cache, update_file, verify)
- `target`: Path or target of the operation
- `command`: Shell command to execute (if applicable)
- `dependencies`: List of operation_ids that must complete first
- `status`: Pending/In-Progress/Completed/Failed
- `rollback_command`: Command to undo this operation (if applicable)

**Execution Sequence**:

1. Create Backup:
   - `operation_id`: 1
   - `operation_name`: "Create Backup Branch"
   - `operation_type`: "backup"
   - `command`: `git branch backup-before-module-cleanup`
   - `dependencies`: []
   - `rollback_command`: `git branch -D backup-before-module-cleanup`

2. Clear Build Cache:
   - `operation_id`: 2
   - `operation_name`: "Clear Docusaurus Build Cache"
   - `operation_type`: "delete_cache"
   - `target`: [`.docusaurus/`, `build/`]
   - `command`: `rm -rf .docusaurus/ build/`
   - `dependencies`: [1]

3. Delete Old Module 3:
   - `operation_id`: 3
   - `operation_name`: "Delete docs/module-03-motion-control/"
   - `operation_type`: "delete_directory"
   - `target`: `docs/module-03-motion-control/`
   - `command`: `rm -rf docs/module-03-motion-control/`
   - `dependencies`: [2]
   - `rollback_command`: `git checkout HEAD -- docs/module-03-motion-control/`

4. Delete Old Module 4:
   - `operation_id`: 4
   - `operation_name`: "Delete docs/module-04-humanoid-integration/"
   - `operation_type`: "delete_directory"
   - `target`: `docs/module-04-humanoid-integration/`
   - `command`: `rm -rf docs/module-04-humanoid-integration/`
   - `dependencies`: [2]
   - `rollback_command`: `git checkout HEAD -- docs/module-04-humanoid-integration/`

5. Update Spec References:
   - `operation_id`: 5
   - `operation_name`: "Update spec file references"
   - `operation_type`: "update_file"
   - `target`: [`specs/002-book-module-structure/tasks.md`, `specs/002-book-module-structure/data-model.md`, etc.]
   - `dependencies`: [3, 4]

6. Test Build:
   - `operation_id`: 6
   - `operation_name`: "Test Documentation Build"
   - `operation_type`: "verify"
   - `command`: `npm run build`
   - `dependencies`: [5]

7. Run Verification Checks:
   - `operation_id`: 7
   - `operation_name`: "Run All Verification Checks"
   - `operation_type`: "verify"
   - `dependencies`: [6]

---

## Relationships

```
Module Directory (1:N) → Reference Location
  "A module directory may be referenced in multiple locations"

Cleanup Operation (1:N) → Verification Check
  "Each cleanup operation may require multiple verification checks"

Cleanup Operation (N:M) → Cleanup Operation (dependencies)
  "Operations have dependency relationships"
```

## Validation Rules

1. **Pre-Deletion Validation**:
   - Module directory must exist before attempting deletion
   - Backup branch must be created successfully before any destructive operations

2. **Deletion Validation**:
   - Only delete directories matching exact paths: `docs/module-03-motion-control/` and `docs/module-04-humanoid-integration/`
   - Do NOT delete: `docs/module-03-isaac-navigation/` or `docs/module-04-vla-multimodal/`

3. **Post-Deletion Validation**:
   - All verification checks must pass before considering cleanup complete
   - Build must succeed with zero errors and zero warnings

4. **Reference Handling**:
   - Historical records (in `history/prompts/`) must be preserved
   - Spec documents should be updated or marked as historical
   - Build cache must be completely removed

## Constraints

- **Atomicity**: Each cleanup operation should be atomic (complete fully or fail fully)
- **Reversibility**: Backup branch must be created before any destructive operations
- **Idempotency**: Running cleanup multiple times should be safe (check existence before deletion)
- **Preservation**: New module directories must never be touched
- **Verification**: All verification checks must pass before considering task complete

## Non-Goals

This data model does NOT include:
- Content migration or merging (out of scope)
- Git history modification (out of scope)
- External bookmark/link updating (out of scope)
- Refactoring or improving new module content (out of scope)
