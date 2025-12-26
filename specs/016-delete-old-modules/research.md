# Research: Delete Old Module 3 and Module 4 Versions

**Feature**: 016-delete-old-modules
**Date**: 2025-12-26
**Phase**: 0 - Research and Discovery

## Research Questions

### Q1: What files and directories need to be deleted?

**Decision**: Delete the following directories and all contents:
- `docs/module-03-motion-control/`
- `docs/module-04-humanoid-integration/`

**Rationale**:
- File size comparison shows old modules have significantly smaller files (2-21KB) compared to new versions (24-67KB)
- Creation dates show old modules created Dec 4, new modules created Dec 25-26
- Content inspection confirms new versions are more comprehensive and complete

**Verification Method**:
```bash
# Old Module 3: 8 files, ~2-15KB each, created Dec 4
ls -la docs/module-03-motion-control/

# Old Module 4: 8 files, ~5-21KB each, created Dec 4
ls -la docs/module-04-humanoid-integration/

# New Module 3: 8+ files, ~28-63KB each, created Dec 25-26
ls -la docs/module-03-isaac-navigation/

# New Module 4: 8+ files, ~24-67KB each, created Dec 26
ls -la docs/module-04-vla-multimodal/
```

**Alternatives Considered**:
- Content merge: Rejected because new versions are complete rewrites with comprehensive coverage
- Git history preservation: Out of scope per spec; history remains in git log

---

### Q2: Where are these modules referenced in the codebase?

**Decision**: References found in the following locations:

1. **Docusaurus Build Cache** (`.docusaurus/` directory)
   - 210+ JSON files containing sidebar and route metadata
   - **Action**: Delete entire `.docusaurus/` directory to force rebuild

2. **Sidebar Configuration** (`sidebars.js`)
   - Uses auto-generated sidebar from directory structure
   - **Action**: No changes needed - sidebar auto-updates after directory deletion

3. **Spec and History Files** (13 files found)
   - `specs/002-book-module-structure/` - references old module names in planning docs
   - `specs/005-book-introduction/` - references old module structure
   - `history/prompts/` - PHR files documenting previous work
   - **Action**: Update spec files; preserve history files (historical record)

4. **Cross-References in Other Modules**
   - `docs/module-02-robot-perception/08-sim-to-real-transfer.md` - may reference Module 3/4
   - **Action**: Search and update any broken cross-references

**Rationale**:
- Auto-generated sidebar means no manual configuration file edits
- Build cache must be cleared to prevent stale references
- Historical records should be preserved but non-functional references updated

**Search Commands Used**:
```bash
# Find all references to old module names
grep -r "module-03-motion-control\|module-04-humanoid-integration" . --include="*.js" --include="*.json" --include="*.md" --include="*.ts"

# Check for static assets
find static/img -name "*module-03*" -o -name "*module-04*"

# Examine build cache
ls -la .docusaurus/
```

**Alternatives Considered**:
- Manual sidebar editing: Not needed due to auto-generation
- Preserving build cache: Rejected - would cause stale reference errors

---

### Q3: Are there shared static assets (images, diagrams) to handle?

**Decision**: Static assets are organized by module in separate directories:
- `static/img/module-03/` - for Module 3 content
- `static/img/module-04/` - for Module 4 content

**Investigation Results**:
```bash
# Module 3 has its own image directory
ls static/img/module-03/
# Output: DIAGRAMS_README.md and various diagrams for isaac-navigation

# Module 4 has its own image directory
ls static/img/module-04/
# Output: DIAGRAMS_README.md and various diagrams for vla-multimodal
```

**Rationale**:
- Each module version has its own dedicated static asset directory
- No shared assets between old and new versions (new modules use module-03/ and module-04/ paths)
- Old modules don't have corresponding static/img directories or use generic paths

**Action Required**:
- Verify old modules don't reference `static/img/module-03/` or `static/img/module-04/`
- If they do, those references will break (acceptable - directories being deleted)
- New modules already use correct paths

**Alternatives Considered**:
- Asset migration: Not needed - new modules have their own assets
- Shared asset directory: Not applicable - no shared assets identified

---

### Q4: How to verify the cleanup is complete and successful?

**Decision**: Multi-step verification process:

1. **Directory Verification**
   ```bash
   # Should return "no such file or directory"
   ls docs/module-03-motion-control/
   ls docs/module-04-humanoid-integration/
   ```

2. **Reference Search**
   ```bash
   # Should return zero results (except in git history and this research doc)
   grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=specs
   grep -r "module-04-humanoid-integration" . --exclude-dir=.git --exclude-dir=specs
   ```

3. **Build Verification**
   ```bash
   # Should complete with zero errors/warnings
   npm run build
   ```

4. **Link Check** (if available)
   ```bash
   # Check for broken links in built site
   npm run build && npx broken-link-checker http://localhost:3000
   ```

5. **Visual Inspection**
   - Start dev server: `npm start`
   - Navigate to Module 3 and Module 4 in sidebar
   - Verify only new versions appear
   - Verify no duplicate entries

**Rationale**:
- Layered verification catches different types of issues
- Automated checks (grep, build) catch reference errors
- Manual inspection catches UX issues (duplicate sidebar entries)

**Alternatives Considered**:
- Automated link checking only: Insufficient - doesn't catch visual/UX issues
- Manual inspection only: Error-prone and time-consuming

---

### Q5: What is the safe deletion sequence to avoid breaking the site?

**Decision**: Execute deletions in this order:

1. **Create backup branch** (safety net)
   ```bash
   git branch backup-before-module-cleanup
   ```

2. **Clear build cache first**
   ```bash
   rm -rf .docusaurus/ build/
   ```

3. **Delete old module directories**
   ```bash
   rm -rf docs/module-03-motion-control/
   rm -rf docs/module-04-humanoid-integration/
   ```

4. **Update spec files** (remove references in planning docs)
   - Update `specs/002-book-module-structure/` files
   - Update `specs/005-book-introduction/` files

5. **Test build immediately**
   ```bash
   npm run build
   ```

6. **Fix any broken references** revealed by build errors

7. **Run full verification** (from Q4)

**Rationale**:
- Backup branch provides rollback capability
- Cache clearing prevents stale reference errors
- Immediate build test catches breaking changes early
- Sequential approach isolates issues

**Alternatives Considered**:
- Delete directories first: Rejected - cache would contain stale references
- No backup: Rejected - violates safety principle
- Batch deletion: Rejected - harder to isolate issues if something breaks

---

## Key Findings Summary

1. **Scope is clear**: Two directories to delete, minimal configuration changes needed
2. **Auto-generated sidebar**: Reduces manual work and error potential
3. **Build cache must be cleared**: Critical to prevent stale references
4. **Verification is straightforward**: Multiple automated checks available
5. **Risk is low**: Backup branch + incremental approach provides safety

## Technology Stack

**Build System**: Docusaurus (Node.js-based static site generator)
**Sidebar**: Auto-generated from directory structure (sidebars.js)
**Build Cache**: `.docusaurus/` and `build/` directories
**Content Format**: Markdown files with frontmatter

## Dependencies

- Node.js and npm (for build commands)
- Git (for version control and backup)
- Bash/PowerShell (for file operations and search)

## Risks and Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Broken cross-references | Medium | Medium | Comprehensive grep search + build verification |
| Stale build cache | High | Medium | Delete cache directories before deletion |
| Broken external bookmarks | Low | Low | Out of scope (documented in spec) |
| Accidental deletion of new modules | Very Low | Critical | Double-check paths before deletion; use backup branch |

## Open Questions

None - all research questions resolved.
