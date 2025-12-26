# Quickstart: Delete Old Module 3 and Module 4 Versions

**Feature**: 016-delete-old-modules
**Estimated Time**: 15-20 minutes
**Prerequisites**: Git, Node.js/npm installed

## One-Line Summary

Remove duplicate/outdated module directories (`module-03-motion-control` and `module-04-humanoid-integration`) and all references, then verify the documentation builds successfully.

## Quick Commands

```bash
# 1. Create backup branch (safety net)
git branch backup-before-module-cleanup

# 2. Clear build cache
rm -rf .docusaurus/ build/

# 3. Delete old module directories
rm -rf docs/module-03-motion-control/
rm -rf docs/module-04-humanoid-integration/

# 4. Search for remaining references (should find minimal results)
grep -r "module-03-motion-control" . --exclude-dir=.git --exclude-dir=specs --exclude-dir=history
grep -r "module-04-humanoid-integration" . --exclude-dir=.git --exclude-dir=specs --exclude-dir=history

# 5. Update any found references in content files
# (Use your text editor to fix any broken cross-references found in step 4)

# 6. Test build
npm run build

# 7. Verify success
# - Check for zero build errors/warnings
# - Verify directories don't exist anymore
test ! -d docs/module-03-motion-control/ && echo "✓ Module 3 removed"
test ! -d docs/module-04-humanoid-integration/ && echo "✓ Module 4 removed"

# 8. Optional: Visual inspection
npm start
# Navigate to Module 3 and Module 4 in the sidebar
# Verify only new versions appear, no duplicates

# 9. Commit changes
git add .
git commit -m "Remove old module 3 and 4 versions

- Deleted docs/module-03-motion-control/
- Deleted docs/module-04-humanoid-integration/
- Cleared build cache
- Updated cross-references in documentation

Resolves: 016-delete-old-modules"
```

## Step-by-Step Guide

### Step 1: Create Safety Backup (1 min)

```bash
# Create a backup branch in case you need to rollback
git branch backup-before-module-cleanup

# Verify it was created
git branch | grep backup-before-module-cleanup
```

**Why**: Provides easy rollback if something goes wrong.

---

### Step 2: Clear Build Cache (1 min)

```bash
# Delete Docusaurus build artifacts
rm -rf .docusaurus/ build/

# Verify deletion
ls -la | grep -E "\.docusaurus|build"
# Should return nothing
```

**Why**: Prevents stale references in cached sidebar/route metadata.

---

### Step 3: Delete Old Module Directories (1 min)

```bash
# Delete old Module 3
rm -rf docs/module-03-motion-control/

# Delete old Module 4
rm -rf docs/module-04-humanoid-integration/

# Verify deletion
ls -la docs/ | grep -E "module-03-motion-control|module-04-humanoid-integration"
# Should return nothing
```

**Why**: Removes the duplicate/outdated content directories.

**Warning**: Double-check you're deleting the correct directories. Do NOT delete:
- `docs/module-03-isaac-navigation/` ❌ (this is the NEW version - keep it!)
- `docs/module-04-vla-multimodal/` ❌ (this is the NEW version - keep it!)

---

### Step 4: Find Remaining References (2-3 min)

```bash
# Search for Module 3 references (excluding git history and specs)
echo "=== Searching for module-03-motion-control ==="
grep -r "module-03-motion-control" . \
  --exclude-dir=.git \
  --exclude-dir=specs \
  --exclude-dir=history \
  --exclude-dir=node_modules \
  --exclude="*.prompt.md"

echo ""
echo "=== Searching for module-04-humanoid-integration ==="
grep -r "module-04-humanoid-integration" . \
  --exclude-dir=.git \
  --exclude-dir=specs \
  --exclude-dir=history \
  --exclude-dir=node_modules \
  --exclude="*.prompt.md"
```

**Expected**: Should find very few or zero results (cross-references in docs might appear).

**Why**: Identifies broken links or references that need updating.

---

### Step 5: Update Remaining References (3-5 min)

If Step 4 found any references in documentation files:

1. Open each file in your text editor
2. Determine if the reference needs updating or removal:
   - **Cross-references to old modules**: Remove or update to point to new modules
   - **Broken links**: Update to new module paths
3. Save changes

**Example**:
```markdown
# Before (broken reference)
See [Module 3](../module-03-motion-control/introduction-to-isaac.md)

# After (updated reference)
See [Module 3](../module-03-isaac-navigation/01-isaac-platform.md)
```

**Note**: References in `specs/` and `history/` directories are acceptable (historical records).

---

### Step 6: Test Documentation Build (3-5 min)

```bash
# Clean build
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build".
```

**If Build Fails**:
1. Read error messages carefully - they usually indicate broken links or references
2. Fix the issues identified
3. Re-run `npm run build`
4. Repeat until successful

---

### Step 7: Verify Cleanup Success (2-3 min)

Run all verification checks:

```bash
# Check 1: Directories don't exist
echo "Checking directory removal..."
if [ -d "docs/module-03-motion-control" ]; then
  echo "❌ FAIL: module-03-motion-control still exists"
else
  echo "✓ PASS: module-03-motion-control removed"
fi

if [ -d "docs/module-04-humanoid-integration" ]; then
  echo "❌ FAIL: module-04-humanoid-integration still exists"
else
  echo "✓ PASS: module-04-humanoid-integration removed"
fi

# Check 2: Build succeeded
echo ""
echo "Checking build status..."
if [ -d "build" ]; then
  echo "✓ PASS: Build directory exists (build succeeded)"
else
  echo "❌ FAIL: Build directory missing (build may have failed)"
fi

# Check 3: New modules still exist (sanity check)
echo ""
echo "Verifying new modules are preserved..."
if [ -d "docs/module-03-isaac-navigation" ]; then
  echo "✓ PASS: module-03-isaac-navigation preserved"
else
  echo "❌ FAIL: module-03-isaac-navigation missing (ERROR - should not happen!)"
fi

if [ -d "docs/module-04-vla-multimodal" ]; then
  echo "✓ PASS: module-04-vla-multimodal preserved"
else
  echo "❌ FAIL: module-04-vla-multimodal missing (ERROR - should not happen!)"
fi
```

All checks should show ✓ PASS.

---

### Step 8: Visual Inspection (2-3 min, optional but recommended)

```bash
# Start development server
npm start
```

**In your browser** (opens automatically at `http://localhost:3000`):
1. Open the sidebar navigation
2. Scroll to Module 3 section
   - Should see: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)" or "module-03-isaac-navigation"
   - Should NOT see: "Module 3: Motion Control" or duplicate entries
3. Scroll to Module 4 section
   - Should see: "Module 4: VLA & Multimodal AI" or "module-04-vla-multimodal"
   - Should NOT see: "Module 4: Vision-Language-Action" old version or duplicates
4. Click through a few Module 3 and Module 4 pages to verify they load correctly

**Press Ctrl+C** to stop the dev server when done.

---

### Step 9: Commit Changes (1-2 min)

```bash
# Stage all changes
git add .

# Review what will be committed
git status

# Create commit
git commit -m "Remove old module 3 and 4 versions

- Deleted docs/module-03-motion-control/ (old version)
- Deleted docs/module-04-humanoid-integration/ (old version)
- Cleared build cache (.docusaurus/ and build/)
- Updated cross-references in documentation
- Verified build succeeds with zero errors

Resolves: 016-delete-old-modules"

# Verify commit
git log -1
```

---

## Rollback (If Needed)

If something goes wrong and you need to restore the old modules:

```bash
# Restore files from backup branch
git checkout backup-before-module-cleanup -- docs/module-03-motion-control/
git checkout backup-before-module-cleanup -- docs/module-04-humanoid-integration/

# Rebuild
rm -rf .docusaurus/ build/
npm run build

# Verify
npm start
```

---

## Success Criteria Checklist

- [ ] Old module directories removed (`module-03-motion-control` and `module-04-humanoid-integration`)
- [ ] Build completes successfully (`npm run build` exits with code 0)
- [ ] Zero build errors or warnings
- [ ] New module directories preserved (`module-03-isaac-navigation` and `module-04-vla-multimodal`)
- [ ] Sidebar shows only new module versions (no duplicates)
- [ ] Minimal or zero references to old modules found (excluding git history and specs)
- [ ] Backup branch created for safety
- [ ] Changes committed to git

---

## Troubleshooting

### Issue: Build fails with "broken link" errors

**Solution**:
1. Read the error message to identify which file has the broken link
2. Open that file and update the link to point to the new module structure
3. Re-run `npm run build`

### Issue: Duplicate entries still appear in sidebar

**Solution**:
1. Delete build cache again: `rm -rf .docusaurus/ build/`
2. Rebuild: `npm run build`
3. Restart dev server: `npm start`

### Issue: Accidentally deleted wrong directory

**Solution**:
1. Use rollback commands above to restore from backup branch
2. Verify correct directories before deletion (double-check paths!)
3. Re-run cleanup process carefully

### Issue: References still found in many files

**Solution**:
- If references are in `specs/` or `history/` directories: This is expected (historical records) - ignore them
- If references are in `docs/` or config files: Update each one individually
- Consider using find-and-replace in your text editor for bulk updates

---

## Time Estimates

- **Fast path** (no issues): 10-12 minutes
- **Standard path** (minor reference updates needed): 15-20 minutes
- **Complex path** (multiple broken references, build issues): 30-45 minutes

## Next Steps

After successful cleanup:
1. Consider running `/sp.tasks` if you need a detailed task breakdown
2. Test the live documentation site thoroughly
3. Inform team members about the cleanup
4. Monitor for any issues reported by users accessing the documentation
