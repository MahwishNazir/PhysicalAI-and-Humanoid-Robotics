# Research: Custom Landing Page in Docusaurus

**Feature**: Landing Page for Physical AI & Humanoid Robotics Book
**Date**: 2025-12-06
**Purpose**: Resolve technical unknowns for implementing a custom landing page at root URL while book content serves from `/intro`

## 1. Docusaurus Custom Pages & Root URL Override

### Decision
Create a React component at `src/pages/index.tsx` while configuring Docusaurus docs to serve from `/intro` instead of `/`.

### Rationale
- Docusaurus uses file-based routing: `src/pages/index.tsx` automatically maps to `/`
- Changing docs `routeBasePath` to `/intro` shifts all book content without breaking internal doc links
- This approach requires minimal configuration and follows Docusaurus conventions
- Allows landing page to exist independently without Docusaurus doc layout (sidebar, breadcrumbs)

### Implementation
1. **Create directory**: `src/pages/` (doesn't exist yet)
2. **Update `docusaurus.config.js`**:
   ```javascript
   docs: {
     routeBasePath: '/intro',  // Changed from '/' (default)
     sidebarPath: './sidebars.js',
   }
   ```
3. **Create `src/pages/index.tsx`**: React component for landing page

### Alternatives Considered
- **Alternative 1**: Use custom plugin to override root route
  - **Rejected**: Adds complexity; Docusaurus's built-in routing is simpler
- **Alternative 2**: Keep docs at `/`, create landing page at `/home`
  - **Rejected**: Users expect landing page at root URL; `/home` is unconventional
- **Alternative 3**: Use iframe or separate static site for landing page
  - **Rejected**: Breaks navigation, doubles build complexity, poor SEO

### Impact on Existing Setup
- Current `docs/intro.md` will be accessible at `/intro` (was `/` before)
- Navbar links referencing docs must update to `/intro/...` URLs
- Sidebar automatically appears only on doc pages, not on custom pages

---

## 2. Routing Strategy for Seamless Navigation

### Decision
Leverage Docusaurus's automatic differentiation between pages and docs for layout rendering.

### How It Works
- **Custom pages** (src/pages/): Use lightweight layout (navbar + custom content + footer)
- **Docs** (docs/): Use full layout with sidebar, breadcrumbs, edit buttons via `@theme/DocPage`
- Docusaurus detects route type and applies appropriate layout automatically
- All routes pre-render to static HTML at build time (no client-side routing delay)

### Navigation Patterns
**From Landing Page → Docs**:
```tsx
// In landing page button component
<Link href="/intro" className="button button--primary">
  Start Reading
</Link>
```

**From Docs → Landing Page**:
```javascript
// In docusaurus.config.js navbar
navbar: {
  title: 'Physical AI & Humanoid Robotics',
  items: [
    {
      to: '/',
      label: 'Home',
      position: 'left',
    },
    // Existing doc links remain
  ],
}
```

### User Experience Flow
1. User visits `/` → sees landing page (no sidebar)
2. User clicks "Start Reading" → navigates to `/intro`
3. Docusaurus detects doc route → renders with sidebar/navigation
4. User clicks "Home" in navbar → returns to `/` → sidebar disappears

### Technical Benefits
- No custom routing logic needed
- Search functionality works across pages + docs
- URL structure is intuitive (root = landing, /intro = book start)
- Static generation = instant page loads

### Alternatives Considered
- **Alternative 1**: Use client-side routing with React Router
  - **Rejected**: Docusaurus already provides routing; adds unnecessary complexity
- **Alternative 2**: Use redirects from `/` → `/intro` with landing page at `/home`
  - **Rejected**: Confusing for users; landing page should be at root

---

## 3. Theme Integration & Design Consistency

### Decision
Use **Infima CSS framework** (Docusaurus's built-in design system) with **CSS Modules** for component-scoped styling.

### Rationale
- Infima provides semantic CSS variables (`--ifm-color-primary`, `--ifm-spacing-*`)
- Automatic light/dark mode support via `[data-theme='dark']` attribute on `<html>` element
- CSS variables automatically switch when user toggles theme (no JavaScript needed)
- CSS Modules prevent style conflicts between landing page and Docusaurus theme
- Brand consistency: landing page uses same color palette as docs

### Light/Dark Mode Implementation
**Completely automatic** - Docusaurus handles theme switching:

```css
/* In component.module.css */
.hero {
  background: var(--ifm-color-primary);
  color: var(--ifm-font-color-base);
  padding: var(--ifm-spacing-vertical);
}

/* Docusaurus automatically adjusts these variables when theme changes:
   - Light mode: --ifm-color-primary = #303F9F (Indigo 700)
   - Dark mode: --ifm-color-primary = lighter variant for contrast
*/
```

**No additional code needed**:
- No `useColorMode()` hook calls required for basic theming
- No manual `className` toggling for dark mode
- CSS variables handle all color transitions

### Key Infima Variables to Use
```css
/* Colors */
--ifm-color-primary: Base brand color
--ifm-color-primary-dark: Darker variant (hover states)
--ifm-color-primary-light: Lighter variant (backgrounds)
--ifm-font-color-base: Body text color (auto-adjusts for dark mode)
--ifm-background-color: Page background

/* Spacing */
--ifm-spacing-horizontal: Standard horizontal spacing (24px)
--ifm-spacing-vertical: Standard vertical spacing (24px)

/* Typography */
--ifm-font-family-base: System font stack
--ifm-font-size-base: 16px (minimum for accessibility)
--ifm-line-height-base: 1.65 (readable line height)
```

### CSS Architecture
```
/src/
  ├── css/
  │   └── custom.css (global overrides for Infima variables)
  └── pages/
      ├── index.tsx
      ├── index.module.css (scoped to landing page)
      └── components/
          ├── Hero.tsx
          ├── Hero.module.css (scoped to Hero component)
          ├── Footer.tsx
          └── Footer.module.css
```

**Global customization** (in `src/css/custom.css`):
```css
:root {
  --ifm-color-primary: #303F9F; /* Indigo 700 */
  --ifm-color-primary-dark: #1E3A8A; /* Indigo 800 */
  --ifm-color-secondary: #00ACC1; /* Cyan 600 */
}

[data-theme='dark'] {
  --ifm-color-primary: #5C6BC0; /* Lighter Indigo for dark mode */
  --ifm-background-color: #121212; /* Dark background */
}
```

### Alternatives Considered
- **Alternative 1**: Use Tailwind CSS
  - **Rejected**: Adds build complexity; Infima already provides utility classes
- **Alternative 2**: Write custom CSS without variables
  - **Rejected**: Would require manual dark mode implementation; inconsistent with docs theme
- **Alternative 3**: Use styled-components or CSS-in-JS
  - **Rejected**: Increases bundle size; CSS Modules are simpler and performant

---

## 4. React Component Structure & Responsive Layout

### Decision
Modular component structure with **CSS Grid + Flexbox** for responsive layout, using mobile-first approach.

### Component Organization
```
/src/pages/
  ├── index.tsx                # Main landing page entry
  ├── index.module.css         # Landing page layout styles
  └── components/
      ├── Hero/
      │   ├── Hero.tsx         # Hero section with title, tagline, CTA
      │   └── Hero.module.css
      ├── Footer/
      │   ├── Footer.tsx       # GitHub, email, copyright
      │   └── Footer.module.css
      └── Button/
          ├── Button.tsx       # Reusable button component (primary/secondary)
          └── Button.module.css
```

### Responsive Layout Approach

**Mobile-First** (default: single column):
```css
.heroContent {
  display: flex;
  flex-direction: column;
  gap: var(--ifm-spacing-vertical);
  align-items: center;
  text-align: center;
}
```

**Desktop** (996px+ breakpoint, Docusaurus standard):
```css
@media (min-width: 996px) {
  .heroContent {
    flex-direction: row; /* Side-by-side layout */
    justify-content: space-between;
    text-align: left;
  }

  .heroImage {
    max-width: 50%; /* Image takes half width on desktop */
  }
}
```

### Infima Button Classes
Use Docusaurus's built-in button styles:

```tsx
// Primary CTA
<button className="button button--primary button--lg">
  Start Reading
</button>

// Secondary auth buttons
<button className="button button--secondary button--outline">
  Sign In
</button>
```

**Available Infima button variants**:
- `button--primary`: Filled button with primary color
- `button--secondary`: Filled button with secondary color
- `button--outline`: Outlined variant (for less prominent actions)
- `button--lg`: Large size (recommended for primary CTA)
- `button--sm`: Small size

### Accessibility Requirements
- **Touch targets**: Minimum 44px height (WCAG 2.1 AA)
  ```css
  .button {
    min-height: 44px;
    padding: 12px 24px;
  }
  ```
- **Text sizing**: Minimum 16px base font size
- **Color contrast**: 4.5:1 for body text, 3:1 for large text (Infima handles this)
- **Keyboard navigation**: All buttons focusable with visible focus indicators

### Responsive Image Handling
```tsx
<img
  src="/img/hero-robot.png"
  alt="Futuristic humanoid robot illustration representing Physical AI"
  className={styles.heroImage}
  loading="lazy"
  decoding="async"
/>
```

```css
.heroImage {
  max-width: 100%;
  height: auto;
  object-fit: contain;
}
```

### Alternatives Considered
- **Alternative 1**: Use CSS Grid for all layout
  - **Rejected**: Flexbox simpler for single-dimensional layouts (column → row)
- **Alternative 2**: Use Docusaurus `@theme/Layout` component
  - **Rejected**: Includes sidebar placeholder; landing page should be simpler
- **Alternative 3**: Use Material-UI or Chakra UI components
  - **Rejected**: Adds large dependencies; Infima provides sufficient styling

---

## 5. Performance Optimization Techniques

### Decision
Leverage Docusaurus's **built-in SSR, code splitting, and intelligent prefetching** without additional optimization libraries.

### How Docusaurus Optimizes Performance Automatically

**1. Static Site Generation (SSG)**:
- All pages pre-rendered to HTML at build time
- No runtime server rendering overhead
- HTML served instantly from CDN or static host
- JavaScript hydrates for interactivity (progressive enhancement)

**2. Automatic Code Splitting**:
- Webpack 5 splits JavaScript per-page
- Landing page bundle doesn't include docs code
- Shared vendor chunks extracted (React, ReactDOM)
- Result: Landing page loads only what it needs (~50-100KB JS)

**3. Intelligent Prefetching** (automatic, no code needed):
Docusaurus prefetches linked pages when:
- **IntersectionObserver**: Link becomes visible in viewport (low priority)
- **onMouseOver**: User hovers over link (high priority)
- **touchstart**: User touches link on mobile (gives parsing time before tap completes)

**For "Start Reading" button**:
```tsx
<Link href="/intro" className="button button--primary">
  Start Reading
</Link>
```
When user hovers, Docusaurus automatically fetches `/intro` chunk. When clicked, page appears instantly.

**4. Image Optimization**:
- Hero images stored in `/static/img/`
- Recommended: PNG/WebP under 100KB
- Use responsive dimensions (800px width max for hero)
- Add attributes for browser optimization:
  ```tsx
  <img
    src="/img/hero-robot.webp"
    loading="lazy"
    decoding="async"
  />
  ```

### Performance Budget
- **HTML**: ~10KB (landing page only)
- **CSS**: ~30KB (Infima + custom styles)
- **JavaScript**: ~100KB (React + ReactDOM + Docusaurus runtime)
- **Images**: ~100KB (optimized hero image)
- **Total**: ~240KB (well under 500KB target)

### Expected Metrics
- **Lighthouse Performance Score**: >90 desktop, >75 mobile
- **First Contentful Paint (FCP)**: <1.5s
- **Largest Contentful Paint (LCP)**: <2.5s (hero image)
- **Time to Interactive (TTI)**: <3.5s

### No Additional Optimization Needed
- ❌ No service workers required
- ❌ No manual lazy loading
- ❌ No custom prefetch logic
- ❌ No image CDN (for MVP)

Docusaurus handles all performance optimizations out-of-the-box.

### Alternatives Considered
- **Alternative 1**: Add custom service worker for offline support
  - **Rejected**: Out of scope for MVP; landing page doesn't need offline access
- **Alternative 2**: Use Next.js Image component for advanced optimization
  - **Rejected**: Requires Next.js; Docusaurus static images are sufficient
- **Alternative 3**: Implement custom prefetching with React Suspense
  - **Rejected**: Docusaurus already provides prefetching

---

## 6. Configuration Changes Required

### docusaurus.config.js

**Change 1: Move docs to /intro**
```javascript
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          routeBasePath: '/intro',  // Changed from '/' (default)
          sidebarPath: './sidebars.js',
        },
      },
    ],
  ],
};
```

**Change 2: Update navbar items**
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
      {
        type: 'doc',
        docId: 'intro',
        position: 'left',
        label: 'Start Reading',
      },
      // Auth buttons can be added here or in custom landing page
    ],
  },
},
```

**Change 3: Custom color scheme** (optional, or keep existing)
```javascript
themeConfig: {
  colorMode: {
    defaultMode: 'light',
    respectPrefersColorScheme: true, // Auto-detect system theme
  },
},
```

In `src/css/custom.css`:
```css
:root {
  --ifm-color-primary: #303F9F;
  --ifm-color-primary-dark: #1E3A8A;
  --ifm-color-secondary: #00ACC1;
}

[data-theme='dark'] {
  --ifm-color-primary: #5C6BC0;
  --ifm-background-color: #121212;
}
```

---

## Implementation Sequence

**Phase 1: Setup**
1. Create `src/pages/` directory
2. Create `src/pages/components/` directory
3. Update `docusaurus.config.js` → change `routeBasePath` to `/intro`
4. Test that docs are accessible at `/intro`

**Phase 2: Landing Page Structure**
5. Create `src/pages/index.tsx` (basic structure)
6. Create `src/pages/index.module.css` (layout)
7. Test that landing page appears at `/`

**Phase 3: Components**
8. Create `Hero` component (title, tagline, image, "Start Reading" button)
9. Create `Footer` component (GitHub, email, copyright)
10. Create `Button` component (primary/secondary variants)

**Phase 4: Styling & Theming**
11. Apply Infima variables in CSS modules
12. Test light/dark mode switching
13. Verify responsive breakpoints (320px, 768px, 996px, 1920px)

**Phase 5: Integration**
14. Add auth button placeholders (Sign In, Sign Up) to Hero
15. Wire "Start Reading" button to `/intro`
16. Test navigation flow: landing → docs → landing

**Phase 6: Polish**
17. Add hero image to `/static/img/`
18. Optimize image size (<100KB)
19. Add alt text for accessibility
20. Run Lighthouse audit (target: >90 desktop, >75 mobile)

---

## Summary of Decisions

| Area | Decision | Key Benefit |
|------|----------|-------------|
| **Custom Pages** | `src/pages/index.tsx` with `routeBasePath: '/intro'` | Simple, follows Docusaurus conventions |
| **Routing** | File-based routing (automatic) | No custom logic needed |
| **Theming** | Infima CSS variables + CSS Modules | Automatic light/dark mode, brand consistency |
| **Components** | Modular React components with scoped styles | Reusable, maintainable, no conflicts |
| **Layout** | CSS Grid + Flexbox, mobile-first | Responsive without complexity |
| **Performance** | Docusaurus built-in SSR, code splitting, prefetching | >90 Lighthouse score achievable without extra work |

---

## Next Steps

With this research complete, proceed to:
1. **Phase 1 Design**: Create `data-model.md` (minimal - no backend), `contracts/` (none needed), `quickstart.md` (component implementation guide)
2. **Task Generation**: Use `/sp.tasks` to break down implementation into actionable items
3. **Implementation**: Execute tasks following the sequence above
