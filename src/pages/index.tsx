/**
 * Landing Page Component
 *
 * This is the main landing page for the Physical AI & Humanoid Robotics book.
 * It serves as the entry point at the root URL (/) and includes:
 *
 * - Hero section with book title, tagline, and "Start Reading" CTA button
 * - Authentication placeholder buttons (Sign In/Sign Up) with coming soon message
 * - Footer with GitHub repository and email contact links
 *
 * The book content is accessible at /intro (configured in docusaurus.config.js).
 *
 * To update content:
 * - Title/Tagline: Edit src/pages/components/Hero/Hero.tsx
 * - Styling: Edit src/css/custom.css for global theme colors
 * - GitHub URL: Update href in src/pages/components/Footer/Footer.tsx
 * - Email: Update href in src/pages/components/Footer/Footer.tsx
 * - Author: Update copyright text in src/pages/components/Footer/Footer.tsx
 */

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
        <Footer />
      </main>
    </Layout>
  );
}
