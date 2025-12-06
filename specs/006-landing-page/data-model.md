# Data Model: Landing Page for Physical AI & Humanoid Robotics Book

**Feature**: Landing Page
**Date**: 2025-12-06
**Note**: This feature is UI-only with no backend/database. This document describes the minimal data structures used in the React components.

## Overview

The landing page is a static React page with no persistent data storage. All content is hardcoded in components or configuration files. The only "data" is:

1. **Static Content**: Text, links, and configuration (hardcoded)
2. **Theme State**: Managed by Docusaurus (light/dark mode preference)
3. **Navigation State**: URL routing handled by Docusaurus

---

## Static Content Configuration

### Landing Page Content (Hardcoded in Components)

**Location**: `src/pages/index.tsx` and child components

```typescript
interface LandingPageContent {
  hero: {
    title: string;              // "Physical AI & Humanoid Robotics"
    tagline: string;            // "Master robot middleware..."
    imageUrl: string;           // "/img/hero-robot.webp"
    imageAlt: string;           // Alt text for accessibility
    ctaPrimary: {
      label: string;            // "Start Reading"
      href: string;             // "/intro"
    };
    ctaAuth: {
      signIn: {
        label: string;          // "Sign In"
        placeholderMessage: string; // "User accounts coming soon..."
      };
      signUp: {
        label: string;          // "Sign Up"
        placeholderMessage: string;
      };
    };
  };
  footer: {
    links: {
      github: {
        label: string;          // "GitHub Repository"
        url: string;            // "https://github.com/username/repo"
        openInNewTab: boolean;  // true
      };
      email: {
        label: string;          // "Contact"
        address: string;        // "contact@example.com"
      };
    };
    copyright: string;          // "© 2025 Your Name"
  };
}
```

**Example Implementation** (in `src/pages/index.tsx`):
```typescript
const landingContent: LandingPageContent = {
  hero: {
    title: "Physical AI & Humanoid Robotics",
    tagline: "Master robot middleware, simulation, AI perception, and voice-controlled autonomy",
    imageUrl: "/img/hero-robot.webp",
    imageAlt: "Futuristic humanoid robot illustration representing Physical AI",
    ctaPrimary: {
      label: "Start Reading",
      href: "/intro",
    },
    ctaAuth: {
      signIn: {
        label: "Sign In",
        placeholderMessage: "User accounts coming soon! For now, click 'Start Reading' to explore the book.",
      },
      signUp: {
        label: "Sign Up",
        placeholderMessage: "User accounts coming soon! For now, click 'Start Reading' to explore the book.",
      },
    },
  },
  footer: {
    links: {
      github: {
        label: "GitHub Repository",
        url: "https://github.com/username/physical-ai-book", // TODO: Update with real URL
        openInNewTab: true,
      },
      email: {
        label: "Contact",
        address: "contact@example.com", // TODO: Update with real email
      },
    },
    copyright: `© ${new Date().getFullYear()} Your Name`, // TODO: Update with real name
  },
};
```

---

## Component Props Interfaces

### Hero Component

```typescript
interface HeroProps {
  title: string;
  tagline: string;
  imageUrl: string;
  imageAlt: string;
  ctaPrimary: {
    label: string;
    href: string;
  };
  ctaAuth: {
    signIn: { label: string; placeholderMessage: string };
    signUp: { label: string; placeholderMessage: string };
  };
}
```

### Footer Component

```typescript
interface FooterProps {
  githubUrl: string;
  githubLabel: string;
  email: string;
  emailLabel: string;
  copyright: string;
}
```

### Button Component

```typescript
interface ButtonProps {
  label: string;
  href?: string;              // For link buttons (primary CTA)
  onClick?: () => void;       // For action buttons (auth placeholders)
  variant: 'primary' | 'secondary' | 'outline';
  size?: 'sm' | 'md' | 'lg';
  disabled?: boolean;
}
```

---

## Theme State (Managed by Docusaurus)

Docusaurus manages theme state automatically. No custom state management needed.

**Location**: HTML `data-theme` attribute
```html
<html data-theme="light">  <!-- or "dark" -->
```

**How It Works**:
- User toggles theme via Docusaurus navbar
- Docusaurus updates `data-theme` attribute on `<html>` element
- CSS variables automatically switch based on `[data-theme='dark']` selector
- Persisted in `localStorage` by Docusaurus

**No React state needed** for theme switching in landing page components.

---

## Navigation State (Docusaurus Router)

**Managed by**: `@docusaurus/router` (wrapper around React Router)

**Routes**:
- `/` → Landing page (`src/pages/index.tsx`)
- `/intro` → Book introduction (`docs/intro.md`)
- `/intro/module-01/...` → Book chapters

**No custom routing logic** - Docusaurus handles all navigation.

---

## Data Validation

### Email Validation

When user clicks email link, browser validates `mailto:` protocol.

```typescript
// In Footer component
<a href={`mailto:${email}`}>
  {emailLabel}
</a>
```

**Browser behavior**:
- Opens default email client
- No validation needed in React component

### URL Validation

GitHub link uses standard `href` attribute.

```typescript
// In Footer component
<a
  href={githubUrl}
  target="_blank"
  rel="noopener noreferrer"
>
  {githubLabel}
</a>
```

**Security**: `rel="noopener noreferrer"` prevents reverse tabnabbing.

---

## Future Data Requirements (Out of Scope for MVP)

When Better Auth is integrated in a future feature, the following data structures will be needed:

### User Entity (Future)

```typescript
interface User {
  id: string;
  email: string;
  name: string;
  createdAt: Date;
  // Progress tracking fields
  readingProgress?: {
    currentChapter: string;
    completedChapters: string[];
    lastReadAt: Date;
  };
}
```

### Authentication State (Future)

```typescript
interface AuthState {
  isAuthenticated: boolean;
  user: User | null;
  loading: boolean;
  error: string | null;
}
```

**Storage**: Will use Better Auth's session management (cookies or JWT)

**Not implemented in MVP** - auth buttons show placeholder message only.

---

## Summary

**Current Data Model**:
- **Static content**: Hardcoded in React components (no database)
- **Theme state**: Managed by Docusaurus (automatic)
- **Navigation state**: Handled by Docusaurus router

**No backend/database needed** for landing page feature.

**Future extensions** (separate features):
- User authentication (Better Auth integration)
- Reading progress tracking (requires database)
- Analytics (user interaction events)
