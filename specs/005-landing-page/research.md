# Landing Page Implementation Research

**Date**: 2025-12-06
**Status**: Complete
**Feature**: Custom Landing Page at Root URL with Docs at `/intro`

---

## 1. Docusaurus Custom Pages & Root URL Override

### Decision
Create a custom landing page at `/src/pages/index.tsx` while shifting docs to `/intro` using route configuration.

### Rationale
- **Docusaurus has two primary content systems**: the docs plugin (for documentation content in `/docs` folder) and the pages plugin (for custom one-off pages in `/src/pages`)
- **File-based routing for pages**: `/src/pages/index.tsx` automatically maps to root URL `/`
- **Cleanest separation**: Custom landing page logic stays isolated in React components, avoiding conflicts with docs plugin
- **Current codebase status**: The project already has `routeBasePath: '/'` for docs. To implement a custom landing page, we must change this to `routeBasePath: '/intro'` to serve docs at `/intro`

### Alternatives Considered
1. **Docs-only mode with slug**: Add `slug: /` to `intro.md` - Works but prevents having a custom landing page
2. **Use markdown page**: Create `/src/pages/index.md` - Less flexible for interactive components and styling
3. **Redirect from home**: Create `/src/pages/index.tsx` that redirects to `/intro` - Adds unnecessary redirect latency

### Code Examples

**Current docusaurus.config.js (needs change)**:
```javascript
// CURRENT - docs serve at root
docs: {
  routeBasePath: '/',
  sidebarPath: './sidebars.js',
}
```

**Recommended change**:
```javascript
// NEW - docs serve at /intro
docs: {
  routeBasePath: '/intro',
  sidebarPath: './sidebars.js',
}
```

**Create landing page at `/src/pages/index.tsx`**:
```typescript
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout title="Physical AI & Humanoid Robotics">
      <main className={styles.main}>
        <Hero />
        <Features />
        <CTA />
      </main>
    </Layout>
  );
}
```

### Critical Implementation Notes
- Must remove or rename any existing `src/pages/index.js/tsx` file
- The `/src/pages/` directory likely doesn't exist yet - must be created
- All relative doc links must be updated to use `/intro/...` instead of `/...`
- Navbar items referencing docs need `href: '/intro/...'`

---

## 2. Routing Strategy for Seamless Navigation

### Decision
Configure Docusaurus routing with:
- Custom landing page at `/`
- Book introduction at `/intro`
- Navbar configuration to show only on docs pages
- Smart linking between pages and docs

### Rationale
- **Page/Docs separation**: Docusaurus naturally isolates pages and docs routing
- **Theme context**: Only docs pages render with `@theme/DocPage` layout (includes sidebar)
- **Conditional navbar rendering**: Can detect route context to hide navbar/sidebar on landing page
- **Static rendering**: Docusaurus pre-renders all routes to static HTML, ensuring fast performance

### Alternatives Considered
1. **Single docs-based approach**: Makes landing page feel like a document with unnecessary sidebar
2. **Separate website**: Creates deployment complexity and navigation issues
3. **Dynamic route hiding**: JavaScript-based navbar hiding adds complexity; CSS/layout approach is cleaner

### Code Examples

**docusaurus.config.js - Navbar configuration**:
```javascript
themeConfig: {
  navbar: {
    title: 'Physical AI & Humanoid Robotics',
    logo: {
      alt: 'Logo',
      src: 'img/logo.svg',
    },
    items: [
      {
        type: 'doc',
        docId: 'intro',
        label: 'Start Reading',
        position: 'left',
      },
      {
        type: 'search',
        position: 'right',
      },
    ],
  },
  docs: {
    sidebar: {
      hideable: true,
      autoCollapseCategories: true,
    },
  },
},
```

**Landing page with CTA button linking to docs**:
```typescript
export function StartReadingButton() {
  return (
    <Link
      className="button button--primary button--lg"
      href="/intro"
    >
      Start Reading
    </Link>
  );
}
```

### Navigation Implementation
- **From landing → docs**: Use `href="/intro"` or `<Link to="/intro">`
- **From docs → landing**: Add navbar link or breadcrumb
- **Sidebar behavior**: Automatically hidden on landing page (not a doc route)
- **Search**: Works across both pages and docs

---

## 3. Theme Integration & Design Consistency

### Decision
Use Docusaurus **Infima CSS framework** with CSS Modules for custom pages, ensuring light/dark mode support and brand consistency.

### Rationale
- **Infima framework**: Docusaurus's built-in design system; provides semantic color variables and utilities
- **CSS custom properties**: `--ifm-color-primary`, `--ifm-spacing-horizontal`, etc. centralized in `/src/css/custom.css`
- **Automatic dark mode**: HTML element has `[data-theme='dark']` attribute; scoping styles handles themes
- **No conflicts**: CSS Modules create scoped class names; no clashes with Docusaurus theme
- **Consistency**: Using Infima ensures landing page feels part of the ecosystem

### Alternatives Considered
1. **Styled Components/Emotion**: Adds runtime overhead; Docusaurus uses static CSS
2. **Tailwind CSS**: Conflicts with Infima; increases bundle size unnecessarily
3. **Global CSS only**: Leads to class name collisions and maintenance issues

### Code Examples

**CSS Module pattern for landing page** (`/src/pages/index.module.css`):
```css
.main {
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  padding: var(--ifm-spacing-vertical) var(--ifm-spacing-horizontal);
  transition: background-color 0.3s, color 0.3s;
}

.hero {
  text-align: center;
  margin: var(--ifm-spacing-vertical) 0;
  background: linear-gradient(135deg, var(--ifm-color-primary), var(--ifm-color-primary-dark));
  color: white;
  padding: calc(var(--ifm-spacing-vertical) * 2);
  border-radius: var(--ifm-border-radius);
}

.features {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: var(--ifm-spacing-horizontal);
}

/* Dark mode automatically handled by Infima variables */
[data-theme='dark'] .hero {
  background: linear-gradient(135deg, var(--ifm-color-primary), var(--ifm-color-primary-dark));
  /* Infima adjusts these vars automatically */
}
```

**Import and use in React**:
```typescript
import styles from './index.module.css';

export function HeroSection() {
  return (
    <section className={styles.hero}>
      <h1>Physical AI & Humanoid Robotics</h1>
      <p>Build robots that think and move</p>
    </section>
  );
}
```

**Custom CSS for theme variables** (`/src/css/custom.css`):
```css
:root {
  --ifm-color-primary: #0066cc;
  --ifm-color-primary-dark: #0052a3;
  --ifm-spacing-vertical: 2rem;
  --ifm-spacing-horizontal: 1rem;
  --ifm-border-radius: 8px;
}

[data-theme='dark'] {
  --ifm-background-color: #1a1a1a;
  --ifm-font-color-base: #e0e0e0;
  --ifm-color-primary: #3399ff;
}
```

### Light/Dark Mode Support
- **Automatic**: Docusaurus handles `prefers-color-scheme` media query
- **Toggle**: Theme switch button in navbar automatically updates `[data-theme]` attribute
- **Infima variables**: All colors automatically adjust; custom CSS variables inherit theme context
- **No extra work**: CSS Modules using Infima vars work in both modes without additional code

---

## 4. React Component Structure & Responsive Layout

### Decision
Organize landing page as modular React components using **CSS Grid + Flexbox** with Docusaurus utilities for responsive behavior.

### Rationale
- **Modular design**: Each section (Hero, Features, CTA, Footer) as separate component for reusability and testability
- **CSS Grid/Flexbox**: No additional framework needed; native browser support; responsive without media queries
- **Mobile-first approach**: 82.9% of traffic is mobile; design from smallest screen first
- **Infima utilities**: Docusaurus provides responsive breakpoints at 996px (mobile/desktop cutoff)
- **Performance**: No JavaScript layout libraries; pure CSS means fast paint and layout

### Alternatives Considered
1. **Docusaurus premade landing page template**: Doesn't exist; would require eject
2. **React Bootstrap**: Adds CSS dependency; Infima already does this
3. **Chakra UI integration**: Runtime overhead; Docusaurus prefers build-time CSS

### Code Examples

**Component structure** (`/src/pages/index.tsx`):
```typescript
import Layout from '@theme/Layout';
import { Hero } from './components/Hero';
import { Features } from './components/Features';
import { CallToAction } from './components/CallToAction';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A comprehensive guide to building humanoid robots"
    >
      <main className={styles.main}>
        <Hero />
        <Features />
        <CallToAction />
      </main>
    </Layout>
  );
}
```

**Hero component** (`/src/pages/components/Hero.tsx`):
```typescript
import Link from '@docusaurus/Link';
import styles from '../styles/Hero.module.css';

export function Hero() {
  return (
    <section className={styles.hero}>
      <div className={styles.container}>
        <h1 className={styles.title}>
          Build Humanoid Robots That Think
        </h1>
        <p className={styles.subtitle}>
          Master embodied AI with ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            href="/intro"
          >
            Start Reading →
          </Link>
          <Link
            className="button button--secondary button--lg"
            href="https://github.com/..."
          >
            View on GitHub
          </Link>
        </div>
      </div>
    </section>
  );
}
```

**Features grid component** (`/src/pages/components/Features.tsx`):
```typescript
import styles from '../styles/Features.module.css';

const FEATURES = [
  {
    title: 'ROS 2 Fundamentals',
    description: 'Master the robotic middleware that powers real robots',
    icon: '⚙️',
  },
  {
    title: 'Digital Twins',
    description: 'Simulate robots safely before deploying to hardware',
    icon: '🤖',
  },
  {
    title: 'AI-Powered Perception',
    description: 'Enable robots to see, understand, and navigate the world',
    icon: '👁️',
  },
  {
    title: 'Voice-Controlled Autonomy',
    description: 'Build robots that respond to natural language commands',
    icon: '🎤',
  },
];

export function Features() {
  return (
    <section className={styles.section}>
      <div className={styles.container}>
        <h2 className={styles.title}>What You'll Learn</h2>
        <div className={styles.grid}>
          {FEATURES.map((feature, idx) => (
            <div key={idx} className={styles.card}>
              <div className={styles.icon}>{feature.icon}</div>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
```

**Features CSS Module** (`/src/pages/styles/Features.module.css`):
```css
.section {
  background: var(--ifm-background-surface-color);
  padding: calc(var(--ifm-spacing-vertical) * 2) var(--ifm-spacing-horizontal);
}

.container {
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 var(--ifm-spacing-horizontal);
}

.title {
  text-align: center;
  font-size: 2rem;
  margin-bottom: var(--ifm-spacing-vertical);
  color: var(--ifm-color-primary);
}

/* Mobile-first: single column */
.grid {
  display: grid;
  grid-template-columns: 1fr;
  gap: var(--ifm-spacing-horizontal);
}

/* Desktop: 2 columns at 996px breakpoint */
@media (min-width: 997px) {
  .grid {
    grid-template-columns: repeat(2, 1fr);
  }
}

/* Large screens: 4 columns */
@media (min-width: 1400px) {
  .grid {
    grid-template-columns: repeat(4, 1fr);
  }
}

.card {
  padding: var(--ifm-spacing-vertical);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-border-radius);
  text-align: center;
  transition: transform 0.2s, box-shadow 0.2s;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 16px rgba(0, 0, 0, 0.1);
}

.icon {
  font-size: 3rem;
  margin-bottom: 1rem;
}

.card h3 {
  margin: var(--ifm-spacing-vertical) 0 0.5rem;
  color: var(--ifm-color-primary);
}

.card p {
  color: var(--ifm-font-color-secondary);
  font-size: 0.95rem;
}
```

### Button Component Pattern
```typescript
// /src/pages/components/Button.tsx
import clsx from 'clsx';
import styles from '../styles/Button.module.css';

type ButtonVariant = 'primary' | 'secondary';
type ButtonSize = 'sm' | 'lg';

interface ButtonProps {
  children: React.ReactNode;
  variant?: ButtonVariant;
  size?: ButtonSize;
  href?: string;
  onClick?: () => void;
  className?: string;
}

export function Button({
  children,
  variant = 'primary',
  size = 'lg',
  href,
  onClick,
  className,
}: ButtonProps) {
  const classes = clsx(
    'button',
    `button--${variant}`,
    `button--${size}`,
    className
  );

  if (href) {
    return <a href={href} className={classes}>{children}</a>;
  }

  return (
    <button onClick={onClick} className={classes}>
      {children}
    </button>
  );
}
```

### Responsive Layout Checklist
- ✅ Mobile-first CSS (single column → multi-column)
- ✅ Flexible fonts (at least 16px base on mobile)
- ✅ Touch-friendly buttons (min 44px height per WCAG)
- ✅ Viewport meta tag (included by Docusaurus automatically)
- ✅ Images responsive (use `max-width: 100%`)
- ✅ No horizontal scroll on mobile

---

## 5. Performance Optimization Techniques

### Decision
Implement **Docusaurus automatic code splitting + image optimization + prefetching** for fast landing page load.

### Rationale
- **Static rendering**: Docusaurus pre-renders all pages to HTML at build time; no runtime compilation
- **Automatic code splitting**: Webpack 5 splits each page's JavaScript separately; landing page code doesn't include docs code
- **Prefetching built-in**: Docusaurus uses IntersectionObserver for low-priority prefetch (when links visible) and onMouseOver for high-priority (when user hovers "Start Reading" button)
- **Mobile prefetching**: Docusaurus prefetches on touchstart for mobile; gives parsing time before tap completes
- **Page speed goal**: Pages loading in 1 second convert 300% more than those loading in 5 seconds

### Alternatives Considered
1. **Manual lazy loading**: Docusaurus handles this automatically; redundant
2. **Service workers**: Docusaurus doesn't use them; static HTML is already fast
3. **Image CDN**: Works but adds cost; local optimization often sufficient

### Code Examples

**Image optimization** - Use responsive images:
```typescript
// Landing page hero image
import HeroImage from '@site/static/img/hero.png';

export function Hero() {
  return (
    <img
      src={HeroImage}
      alt="Humanoid robot"
      style={{
        maxWidth: '100%',
        height: 'auto',
        width: '100%',
      }}
      loading="lazy"
      decoding="async"
    />
  );
}
```

**Store images optimized**:
- Images in `/static/img/` should be:
  - PNG/WebP (JPG for photos)
  - Under 100KB for hero image (use compression tools)
  - Mobile: ~600px wide, Desktop: ~1200px wide
  - Provide 2x variants for retina displays

**Docusaurus automatically prefetches** - No code needed:
```javascript
// Docusaurus core automatically adds:
// 1. IntersectionObserver on visible links (low-priority)
// 2. onMouseOver listeners for high-priority prefetch
// 3. touchstart for mobile
// Result: When user clicks "Start Reading", chunk is usually already loaded
```

**Async component loading** (if needed for large features):
```typescript
import { Suspense, lazy } from 'react';

const HeavyFeatureSection = lazy(() =>
  import('./components/HeavyFeatures').then(m => ({
    default: m.HeavyFeatures,
  }))
);

export default function Home() {
  return (
    <>
      <Hero />
      <Suspense fallback={<div>Loading features...</div>}>
        <HeavyFeatureSection />
      </Suspense>
    </>
  );
}
```

### Performance Checklist
- ✅ Hero image under 100KB
- ✅ No inline scripts on landing page (Docusaurus handles SSR)
- ✅ CSS Modules (already scoped, no duplication)
- ✅ Lazy loading images and non-critical sections
- ✅ Prefetch to `/intro` automatically triggers when hovering "Start Reading"
- ✅ Use native `loading="lazy"` and `decoding="async"` on images

### Measuring Performance
```bash
# After deployment
npm run build
npm run serve

# Then test with:
# - Chrome DevTools Lighthouse
# - WebPageTest (webpagetest.org)
# - GTmetrix
# Goal: Lighthouse > 90 for Performance
```

---

## 6. Summary & Implementation Order

### Key Decisions
| Area | Decision |
|------|----------|
| **Root Page** | Create `/src/pages/index.tsx` React component |
| **Docs Location** | Change `routeBasePath` from `/` to `/intro` |
| **Styling** | CSS Modules + Infima design system |
| **Dark Mode** | Automatic via Infima `[data-theme]` attribute |
| **Components** | Modular (Hero, Features, CTA, Footer) |
| **Layout** | CSS Grid/Flexbox, mobile-first |
| **Performance** | Rely on Docusaurus SSR + built-in prefetching |

### Implementation Sequence
1. Update `docusaurus.config.js`: Change docs `routeBasePath` to `/intro`
2. Create `/src/pages/` directory structure
3. Create `/src/pages/index.tsx` with Layout wrapper
4. Build component architecture (Hero, Features, CTA)
5. Create CSS Modules with Infima variables
6. Test responsive behavior (mobile, tablet, desktop)
7. Verify dark mode toggle works
8. Test prefetching to `/intro` on button hover
9. Run Lighthouse audit; target >90 performance score

### Non-Goals
- Custom navigation sidebar on landing page (kept clean)
- Blog or other content plugins (docs + pages only)
- Eject or swizzle theme components (use CSS instead)

---

## Research Sources

- [Docusaurus: Creating Pages](https://docusaurus.io/docs/creating-pages)
- [Docusaurus: Styling and Layout](https://docusaurus.io/docs/styling-layout)
- [Docusaurus: Swizzling](https://docusaurus.io/docs/advanced/swizzling)
- [Docusaurus: Static Assets](https://docusaurus.io/docs/static-assets)
- [Docusaurus: Configuration](https://docusaurus.io/docs/configuration)
- [Docusaurus: Advanced Routing](https://docusaurus.io/docs/advanced/routing)
- [Stack Overflow: Disable Default Landing Page](https://stackoverflow.com/questions/78033053/how-can-i-disable-the-my-site-landing-page-in-docusaurus)
- [Stack Overflow: Redirect to Docs](https://stackoverflow.com/questions/58665817/redirect-to-docs-from-landing-page-in-docusaurus-v2)
- [Docusaurus: Client Architecture](https://docusaurus.io/docs/docusaurus-core)
- [Responsive Design Landing Page: 10 Powerful Tips for 2025](https://lineardesign.com/blog/responsive-design-landing-page/)
- [React Button Components in Docusaurus](https://stackoverflow.com/questions/69173994/docusaurus-2-how-to-add-custom-react-component-in-navbar)
- [GitHub: Docusaurus Prefetch PR #3723](https://github.com/facebook/docusaurus/pull/3723)
- [GitHub: Docusaurus Mobile Prefetch PR #8109](https://github.com/facebook/docusaurus/pull/8109)
