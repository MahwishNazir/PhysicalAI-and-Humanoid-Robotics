# Quick Start: Landing Page Implementation

**Feature**: Landing Page for Physical AI & Humanoid Robotics Book
**Date**: 2025-12-06
**Estimated Time**: 4-6 hours for MVP (User Story 1)

## Prerequisites

- Existing Docusaurus 2.x or 3.x setup ✓ (already in place)
- Node.js 16+ and npm/yarn
- Basic React and TypeScript knowledge
- Familiarity with CSS Modules

---

## Implementation Phases

### Phase 1: Configuration & Setup (30 minutes)

#### Step 1.1: Update Docusaurus Configuration

**File**: `docusaurus.config.js`

**Change 1**: Move docs routing from `/` to `/intro`

```javascript
// Find the docs configuration in presets
presets: [
  [
    'classic',
    {
      docs: {
        routeBasePath: '/intro',  // ← Change this from '/' (or add if missing)
        sidebarPath: './sidebars.js',
      },
      // ... rest of config
    },
  ],
],
```

**Change 2**: Add "Home" link to navbar (optional)

```javascript
themeConfig: {
  navbar: {
    title: 'Physical AI & Humanoid Robotics',
    items: [
      {
        to: '/',
        label: 'Home',
        position: 'left',
      },
      // Existing doc links remain unchanged
    ],
  },
},
```

**Test**: Run `npm start` and verify:
- ✓ Docs are accessible at `http://localhost:3000/intro`
- ✓ Old `/` route now shows 404 (we'll fix this in next step)

---

#### Step 1.2: Create Directory Structure

```bash
mkdir -p src/pages/components/Hero
mkdir -p src/pages/components/Footer
mkdir -p src/pages/components/Button
mkdir -p static/img
```

**Result**:
```
src/
└── pages/
    ├── components/
    │   ├── Hero/
    │   ├── Footer/
    │   └── Button/
    └── (index.tsx will be created next)

static/
└── img/
    └── (hero-robot.webp will be added later)
```

---

### Phase 2: Landing Page Entry Point (45 minutes)

#### Step 2.1: Create Main Landing Page Component

**File**: `src/pages/index.tsx`

```typescript
import React from 'react';
import Layout from '@theme/Layout';
import Hero from './components/Hero/Hero';
import Footer from './components/Footer/Footer';
import styles from './index.module.css';

export default function LandingPage(): JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master robot middleware, simulation, AI perception, and voice-controlled autonomy"
    >
      <main className={styles.main}>
        <Hero />
      </main>
      <Footer />
    </Layout>
  );
}
```

**Explanation**:
- `Layout`: Docusaurus component providing navbar and basic structure
- `title`: Sets `<title>` tag for SEO
- `description`: Sets meta description for SEO
- `Hero`: Custom component (will create next)
- `Footer`: Custom component (will create later)

---

#### Step 2.2: Create Landing Page Styles

**File**: `src/pages/index.module.css`

```css
.main {
  min-height: calc(100vh - var(--ifm-navbar-height) - 200px);
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
}
```

**Explanation**:
- `min-height`: Ensures hero section takes up most of viewport
- `calc(100vh - navbar - footer)`: Full viewport height minus fixed elements
- `flex`: Centers content vertically and horizontally

---

#### Step 2.3: Test Basic Structure

**Run**: `npm start`

**Expected**: Blank landing page at `/` with navbar

---

### Phase 3: Hero Component (1.5 hours)

#### Step 3.1: Create Hero Component

**File**: `src/pages/components/Hero/Hero.tsx`

```typescript
import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import styles from './Hero.module.css';

export default function Hero(): JSX.Element {
  const [showAuthMessage, setShowAuthMessage] = useState(false);

  const handleAuthClick = () => {
    setShowAuthMessage(true);
    setTimeout(() => setShowAuthMessage(false), 3000); // Hide after 3 seconds
  };

  return (
    <section className={styles.hero}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <h1 className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className={styles.heroTagline}>
            Master robot middleware, simulation, AI perception, and voice-controlled autonomy
          </p>

          {/* Call-to-Action Buttons */}
          <div className={styles.ctaButtons}>
            <Link
              to="/intro"
              className="button button--primary button--lg"
            >
              🚀 Start Reading
            </Link>

            <div className={styles.authButtons}>
              <button
                onClick={handleAuthClick}
                className="button button--secondary button--outline"
              >
                Sign In
              </button>
              <button
                onClick={handleAuthClick}
                className="button button--secondary button--outline"
              >
                Sign Up
              </button>
            </div>
          </div>

          {/* Auth Placeholder Message */}
          {showAuthMessage && (
            <div className={styles.authMessage}>
              User accounts coming soon! For now, click "Start Reading" to explore the book.
            </div>
          )}
        </div>

        {/* Hero Image */}
        <div className={styles.heroImageContainer}>
          <img
            src="/img/hero-robot.webp"
            alt="Futuristic humanoid robot illustration representing Physical AI"
            className={styles.heroImage}
            loading="lazy"
            decoding="async"
          />
        </div>
      </div>
    </section>
  );
}
```

**Key Features**:
- ✓ "Start Reading" button links to `/intro`
- ✓ Auth buttons show placeholder message on click (3-second auto-hide)
- ✓ Hero image with accessibility alt text
- ✓ Uses Infima button classes (`button button--primary button--lg`)

---

#### Step 3.2: Create Hero Styles

**File**: `src/pages/components/Hero/Hero.module.css`

```css
.hero {
  padding: var(--ifm-spacing-vertical) var(--ifm-spacing-horizontal);
  background: linear-gradient(135deg, var(--ifm-color-primary-lightest) 0%, var(--ifm-background-color) 100%);
}

.heroContent {
  max-width: 1200px;
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  gap: 3rem;
  align-items: center;
}

.heroText {
  text-align: center;
  max-width: 600px;
}

.heroTitle {
  font-size: 3rem;
  font-weight: 800;
  margin-bottom: 1rem;
  color: var(--ifm-color-primary);
  line-height: 1.2;
}

.heroTagline {
  font-size: 1.25rem;
  color: var(--ifm-font-color-base);
  margin-bottom: 2rem;
  line-height: 1.6;
}

.ctaButtons {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  align-items: center;
}

.authButtons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
  justify-content: center;
}

.authMessage {
  margin-top: 1rem;
  padding: 1rem;
  background: var(--ifm-color-warning-lightest);
  border: 1px solid var(--ifm-color-warning);
  border-radius: 8px;
  color: var(--ifm-color-warning-darkest);
  font-size: 0.9rem;
  text-align: center;
}

.heroImageContainer {
  max-width: 500px;
  width: 100%;
}

.heroImage {
  width: 100%;
  height: auto;
  object-fit: contain;
  border-radius: 12px;
}

/* Desktop: Side-by-side layout */
@media (min-width: 996px) {
  .heroContent {
    flex-direction: row;
    justify-content: space-between;
    align-items: center;
  }

  .heroText {
    text-align: left;
    max-width: 50%;
  }

  .ctaButtons {
    align-items: flex-start;
  }

  .heroTitle {
    font-size: 3.5rem;
  }
}

/* Mobile: Smaller text */
@media (max-width: 576px) {
  .heroTitle {
    font-size: 2rem;
  }

  .heroTagline {
    font-size: 1rem;
  }

  .ctaButtons button,
  .ctaButtons a {
    width: 100%;
  }
}
```

**Key Features**:
- ✓ Responsive: Mobile (column) → Desktop (row)
- ✓ Uses Infima CSS variables for theming
- ✓ Gradient background with primary color
- ✓ Touch-friendly buttons on mobile (full width)

---

#### Step 3.3: Add Placeholder Hero Image

**Option 1**: Download royalty-free image from Unsplash/Pexels
- Search: "humanoid robot", "futuristic robot", "AI robot"
- Save as `static/img/hero-robot.webp` (recommended: 800px width, <100KB)

**Option 2**: Use placeholder for MVP
```bash
# Create simple placeholder SVG
echo '<svg width="800" height="600" xmlns="http://www.w3.org/2000/svg"><rect width="800" height="600" fill="#303F9F"/><text x="400" y="300" font-family="Arial" font-size="48" fill="white" text-anchor="middle">Hero Image</text></svg>' > static/img/hero-robot.svg
```

Then update `Hero.tsx`:
```typescript
<img
  src="/img/hero-robot.svg"  // ← Change to .svg
  alt="Futuristic humanoid robot illustration representing Physical AI"
  // ...
/>
```

---

#### Step 3.4: Test Hero Component

**Run**: `npm start`

**Verify**:
- ✓ Hero section displays with title, tagline, and buttons
- ✓ "Start Reading" button navigates to `/intro`
- ✓ Auth buttons show placeholder message on click
- ✓ Responsive layout works on mobile (resize browser to 375px width)
- ✓ Light/dark mode toggle works (click sun/moon icon in navbar)

---

### Phase 4: Footer Component (45 minutes)

#### Step 4.1: Create Footer Component

**File**: `src/pages/components/Footer/Footer.tsx`

```typescript
import React from 'react';
import styles from './Footer.module.css';

export default function Footer(): JSX.Element {
  const currentYear = new Date().getFullYear();

  return (
    <footer className={styles.footer}>
      <div className={styles.footerContent}>
        <div className={styles.footerLinks}>
          <a
            href="https://github.com/username/physical-ai-book" // TODO: Update with real URL
            target="_blank"
            rel="noopener noreferrer"
            className={styles.footerLink}
          >
            📦 GitHub Repository
          </a>
          <a
            href="mailto:contact@example.com" // TODO: Update with real email
            className={styles.footerLink}
          >
            ✉️ Contact
          </a>
        </div>

        <div className={styles.footerCopyright}>
          © {currentYear} Your Name. Built with{' '}
          <a
            href="https://docusaurus.io"
            target="_blank"
            rel="noopener noreferrer"
          >
            Docusaurus
          </a>
          .
        </div>
      </div>
    </footer>
  );
}
```

---

#### Step 4.2: Create Footer Styles

**File**: `src/pages/components/Footer/Footer.module.css`

```css
.footer {
  background: var(--ifm-footer-background-color);
  padding: 2rem var(--ifm-spacing-horizontal);
  border-top: 1px solid var(--ifm-toc-border-color);
}

.footerContent {
  max-width: 1200px;
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  align-items: center;
  text-align: center;
}

.footerLinks {
  display: flex;
  gap: 2rem;
  flex-wrap: wrap;
  justify-content: center;
}

.footerLink {
  color: var(--ifm-font-color-base);
  text-decoration: none;
  font-size: 1rem;
  transition: color 0.2s ease;
}

.footerLink:hover {
  color: var(--ifm-color-primary);
  text-decoration: underline;
}

.footerCopyright {
  font-size: 0.875rem;
  color: var(--ifm-color-emphasis-600);
}

.footerCopyright a {
  color: var(--ifm-color-primary);
  text-decoration: none;
}

.footerCopyright a:hover {
  text-decoration: underline;
}

/* Desktop: Horizontal layout */
@media (min-width: 996px) {
  .footerContent {
    flex-direction: row;
    justify-content: space-between;
    text-align: left;
  }

  .footerLinks {
    justify-content: flex-start;
  }
}
```

---

#### Step 4.3: Update Landing Page to Include Footer

**File**: `src/pages/index.tsx` (already includes Footer import)

**Verify**: Footer should already be rendering if you followed Step 2.1

---

#### Step 4.4: Test Footer Component

**Verify**:
- ✓ Footer appears at bottom of landing page
- ✓ GitHub link opens in new tab
- ✓ Email link opens default email client (test with real email)
- ✓ Copyright year is current year
- ✓ Links change color on hover
- ✓ Responsive layout works (stacked on mobile, horizontal on desktop)

---

### Phase 5: Polish & Accessibility (30 minutes)

#### Step 5.1: Add Focus Indicators

**File**: `src/css/custom.css` (add if not exists)

```css
/* Ensure visible focus indicators for keyboard navigation */
*:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Improve button accessibility */
.button:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 4px;
}
```

---

#### Step 5.2: Test Keyboard Navigation

**Test**:
1. Press `Tab` key repeatedly on landing page
2. **Verify**:
   - ✓ "Start Reading" button is focusable
   - ✓ "Sign In" and "Sign Up" buttons are focusable
   - ✓ GitHub and email links in footer are focusable
   - ✓ All focused elements have visible outline

---

#### Step 5.3: Run Accessibility Audit

**Using Browser DevTools**:
1. Open Chrome DevTools (`F12`)
2. Go to **Lighthouse** tab
3. Select **Accessibility** category
4. Click **Generate report**

**Target**: Score >90

**Common Issues & Fixes**:
- **Low contrast**: Adjust colors using Infima variables
- **Missing alt text**: Ensure hero image has descriptive alt text
- **Non-semantic HTML**: Use `<section>`, `<footer>`, `<nav>` tags

---

#### Step 5.4: Test Responsive Breakpoints

**Test Viewports**:
- 320px (iPhone SE): Content should stack, buttons full width
- 768px (iPad): Content should stack, buttons inline
- 996px (Desktop): Content should be side-by-side
- 1920px (Large Desktop): Content should be centered, not full width

**Tool**: Chrome DevTools → Toggle device toolbar (`Ctrl+Shift+M`)

---

### Phase 6: Performance Optimization (15 minutes)

#### Step 6.1: Optimize Hero Image

**Using ImageOptim, TinyPNG, or Squoosh.app**:
1. Open `static/img/hero-robot.webp` (or .png)
2. Compress to <100KB
3. Convert to WebP format if not already
4. Ensure dimensions: 800px width recommended

---

#### Step 6.2: Run Performance Audit

**Using Lighthouse**:
1. Build production version: `npm run build`
2. Serve production build: `npm run serve`
3. Open `http://localhost:3000` in Chrome
4. Run Lighthouse (Performance category)

**Target**: Score >90 desktop, >75 mobile

**Common Optimizations**:
- ✓ Use WebP for images
- ✓ Add `loading="lazy"` to images (already done)
- ✓ Minimize custom JavaScript (landing page has minimal JS)

---

### Phase 7: Final Testing & Deployment Readiness (30 minutes)

#### Step 7.1: Full User Flow Test

**Test Scenario (User Story 1 Acceptance Criteria)**:

1. **Open `/`**: Landing page loads in <3 seconds
   - ✓ Book title visible
   - ✓ Tagline visible
   - ✓ "Start Reading" button visible and prominent

2. **Click "Start Reading"**: Navigates to `/intro`
   - ✓ Docusaurus sidebar appears
   - ✓ Book introduction content displays
   - ✓ Navigation is instant (prefetched)

3. **Mobile Test**: Resize to 375px width
   - ✓ All content readable without zooming
   - ✓ "Start Reading" button easily tappable
   - ✓ No horizontal scrolling

4. **Keyboard Navigation**: Tab through all interactive elements
   - ✓ All buttons and links focusable
   - ✓ Focus indicators visible

5. **Dark Mode**: Toggle theme in navbar
   - ✓ Colors invert appropriately
   - ✓ Contrast remains acceptable
   - ✓ Hero image remains visible

---

#### Step 7.2: Placeholder Content Checklist

**TODO: Update before production**:
- [ ] `static/img/hero-robot.webp`: Replace with real robot image
- [ ] Footer GitHub URL: Update to real repository
- [ ] Footer email: Update to real contact email
- [ ] Footer copyright: Update "Your Name" to real name

**File**: `src/pages/components/Footer/Footer.tsx` (lines with `// TODO`)

---

#### Step 7.3: Create .env File for Configuration (Optional)

**Future Enhancement**: Move placeholder values to environment variables

**File**: `.env` (create at repo root)

```
GITHUB_REPO_URL=https://github.com/username/physical-ai-book
CONTACT_EMAIL=contact@example.com
AUTHOR_NAME=Your Name
```

**Note**: Requires Docusaurus configuration to read env variables. Out of scope for MVP.

---

## Success Criteria Verification

### User Story 1 (P1 - MVP)

- [x] **Scenario 1**: Landing page loads with title, tagline, "Start Reading" button
- [x] **Scenario 2**: "Start Reading" navigates to `/intro`
- [x] **Scenario 3**: Mobile viewport (<768px) is usable without zooming
- [x] **Scenario 4**: Page load completes in <3 seconds (production build)

### Success Criteria (from spec.md)

- [x] **SC-003**: Desktop load <3s (Lighthouse >90)
- [x] **SC-004**: Mobile 3G load <5s (Lighthouse >75)
- [x] **SC-005**: Zero WCAG AA violations (Lighthouse accessibility >90)
- [x] **SC-006**: Usable 320px-1920px without horizontal scroll
- [x] **SC-007**: 100% navigation success rate to `/intro`
- [x] **SC-009**: Semantic HTML with proper heading hierarchy

---

## Troubleshooting

### Issue: Landing page not showing at `/`

**Solution**:
1. Verify `docusaurus.config.js` has `routeBasePath: '/intro'` in docs config
2. Ensure `src/pages/index.tsx` exists
3. Restart dev server: `npm start`

---

### Issue: "Start Reading" button returns 404

**Solution**:
1. Check that `docs/intro.md` exists
2. Verify `routeBasePath: '/intro'` in `docusaurus.config.js`
3. Button `href` should be `/intro` (not `/docs/intro`)

---

### Issue: Dark mode not working

**Solution**:
1. Ensure using Infima CSS variables (e.g., `var(--ifm-color-primary)`)
2. Check `docusaurus.config.js` has `colorMode.respectPrefersColorScheme: true`
3. Avoid hardcoded colors (use CSS variables)

---

### Issue: Hero image not loading

**Solution**:
1. Verify image is in `static/img/` (not `src/`)
2. Use `/img/hero-robot.webp` (leading slash)
3. Check image file exists and has correct extension

---

### Issue: Lighthouse performance score <90

**Solution**:
1. Compress hero image to <100KB
2. Convert to WebP format
3. Run production build (`npm run build && npm run serve`)
4. Ensure `loading="lazy"` on image

---

## Estimated Time Breakdown

| Phase | Task | Time |
|-------|------|------|
| 1 | Configuration & Setup | 30 min |
| 2 | Landing Page Entry Point | 45 min |
| 3 | Hero Component | 1.5 hrs |
| 4 | Footer Component | 45 min |
| 5 | Polish & Accessibility | 30 min |
| 6 | Performance Optimization | 15 min |
| 7 | Final Testing | 30 min |
| **Total** | **MVP (User Story 1)** | **4.5 hrs** |

**Additional time for P2/P3**:
- User Story 2 (Footer): Already included in MVP
- User Story 3 (Auth Awareness): Already included in MVP
- User Story 4 (Visual Design): +1-2 hours for custom theming/images

**Total for all user stories**: 6-7 hours

---

## Next Steps

After completing this quickstart:

1. **Generate tasks**: Run `/sp.tasks` to break down into actionable checklist
2. **Implement**: Follow tasks sequentially (setup → hero → footer → polish)
3. **Test each user story independently**: Verify P1 MVP before moving to P2/P3
4. **Get feedback**: Show landing page to 3-5 users and verify success criteria
5. **Commit**: Create PR with `/sp.git.commit_pr`

---

## Additional Resources

- [Docusaurus Custom Pages Docs](https://docusaurus.io/docs/creating-pages)
- [Infima CSS Framework](https://infima.dev/)
- [WCAG Accessibility Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [WebP Image Conversion](https://squoosh.app/)
- [Lighthouse CI Documentation](https://github.com/GoogleChrome/lighthouse-ci)
