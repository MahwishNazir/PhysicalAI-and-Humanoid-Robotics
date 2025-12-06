import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './Hero.module.css';

export default function Hero(): JSX.Element {
  const [showAuthMessage, setShowAuthMessage] = useState(false);

  const handleAuthClick = () => {
    setShowAuthMessage(true);
    setTimeout(() => {
      setShowAuthMessage(false);
    }, 3000);
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
            src={useBaseUrl('/img/hero-robot.svg')}
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
