# Feature Specification: Delete Old Module 3 and Module 4 Versions

**Feature Branch**: `016-delete-old-modules`
**Created**: 2025-12-26
**Status**: Draft
**Input**: User description: "i want to delete the old version of module 3 and module 4."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Remove Duplicate Module Directories (Priority: P1)

The repository contains duplicate module directories for Module 3 and Module 4. The older versions (`module-03-motion-control` and `module-04-humanoid-integration`) need to be removed to eliminate confusion and maintain a single source of truth. The newer versions (`module-03-isaac-navigation` and `module-04-vla-multimodal`) contain more comprehensive and up-to-date content.

**Why this priority**: Duplicate content creates confusion for users, increases maintenance burden, and can lead to inconsistent documentation. This cleanup is critical for repository health and user experience.

**Independent Test**: Can be fully tested by verifying that the old module directories no longer exist in the file system and that all references to them have been removed or updated.

**Acceptance Scenarios**:

1. **Given** the repository contains `docs/module-03-motion-control/` and `docs/module-04-humanoid-integration/` directories, **When** the cleanup is executed, **Then** these directories are completely removed from the file system
2. **Given** the Docusaurus configuration and sidebar references the old module directories, **When** the cleanup is executed, **Then** all references are removed or updated to point only to the new versions
3. **Given** the repository may have other references to old modules (specs, images, build files), **When** the cleanup is executed, **Then** all such references are identified and removed

---

### User Story 2 - Verify Documentation Build Integrity (Priority: P2)

After removing the old modules, ensure that the documentation site builds successfully and that navigation remains intact, with no broken links or missing pages.

**Why this priority**: Broken builds or navigation issues would render the documentation unusable. This verification step ensures that the cleanup doesn't introduce regressions.

**Independent Test**: Can be tested independently by running the documentation build process and verifying no errors, warnings, or broken links exist.

**Acceptance Scenarios**:

1. **Given** the old modules have been removed, **When** running the documentation build command, **Then** the build completes successfully with no errors
2. **Given** the documentation site is built, **When** navigating through Module 3 and Module 4 sections, **Then** all links work correctly and point to the new module versions
3. **Given** the sidebar configuration has been updated, **When** viewing the documentation site, **Then** only the new module versions appear in the navigation

---

### Edge Cases

- What happens if there are cross-references between old and new module versions?
- How does the system handle static assets (images, diagrams) that may be shared between old and new versions?
- What if the Docusaurus build cache contains references to deleted files?
- Are there any git submodules or external references that point to the old directories?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST delete the `docs/module-03-motion-control/` directory and all its contents
- **FR-002**: System MUST delete the `docs/module-04-humanoid-integration/` directory and all its contents
- **FR-003**: System MUST identify and remove all references to `module-03-motion-control` in configuration files (e.g., `docusaurus.config.js`, sidebar configurations)
- **FR-004**: System MUST identify and remove all references to `module-04-humanoid-integration` in configuration files
- **FR-005**: System MUST verify that no broken links exist after deletion by running the documentation build
- **FR-006**: System MUST check for and remove any orphaned static assets (images, diagrams) that were only used by the old modules
- **FR-007**: System MUST update or remove any spec files or task files that reference the old module directories
- **FR-008**: System MUST preserve all content in `docs/module-03-isaac-navigation/` and `docs/module-04-vla-multimodal/` directories
- **FR-009**: System MUST clear any build caches that might contain references to deleted files

### Key Entities *(include if feature involves data)*

- **Old Module 3**: Directory `docs/module-03-motion-control/` containing 8 markdown files with older, less comprehensive content (created Dec 4)
- **Old Module 4**: Directory `docs/module-04-humanoid-integration/` containing 8 markdown files with older, less comprehensive content (created Dec 4)
- **New Module 3**: Directory `docs/module-03-isaac-navigation/` containing updated, comprehensive chapter files (created Dec 25-26)
- **New Module 4**: Directory `docs/module-04-vla-multimodal/` containing updated, comprehensive chapter files (created Dec 26)
- **Configuration Files**: `docusaurus.config.js`, sidebar configuration files, and build configuration
- **Static Assets**: Images, diagrams, and other resources in `static/img/` directories

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Old module directories (`module-03-motion-control` and `module-04-humanoid-integration`) are completely removed from the repository
- **SC-002**: Documentation build completes successfully with zero errors and zero warnings
- **SC-003**: All navigation links in the documentation site work correctly (0 broken links)
- **SC-004**: Sidebar shows only the new module versions with no duplicate or orphaned entries
- **SC-005**: Running a repository-wide search for "module-03-motion-control" and "module-04-humanoid-integration" returns zero results (except in git history)

## Assumptions

- The newer module versions (`module-03-isaac-navigation` and `module-04-vla-multimodal`) are the canonical versions to keep
- No external documentation or systems depend on the specific URLs of the old module pages
- The old modules share no unique content that needs to be preserved (all valuable content exists in new versions)
- Build cache can be safely cleared without affecting other parts of the system

## Out of Scope

- Migration of any unique content from old modules to new ones (assumes new versions are complete)
- Updating external links or bookmarks that users may have created
- Modifying git history to remove old commits that added these directories
- Refactoring or improving the content of the new module versions
