import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import styles from './ThemeSwitcher.module.css';

type ThemeSwitcherProps = {
  className?: string;
};

export default function ThemeSwitcher({ className }: ThemeSwitcherProps): JSX.Element {
  const [theme, setTheme] = useState<'light' | 'dark' | 'auto'>('auto');
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);

    // Get saved theme from localStorage or use system preference
    const savedTheme = localStorage.getItem('theme') as 'light' | 'dark' | 'auto' | null;
    const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';

    const initialTheme = savedTheme || 'auto';
    setTheme(initialTheme);

    // Apply theme immediately after mount
    applyTheme(initialTheme, systemTheme);
  }, []);

  const applyTheme = (selectedTheme: 'light' | 'dark' | 'auto', systemTheme?: 'light' | 'dark') => {
    const actualTheme = selectedTheme === 'auto' ? (systemTheme || 'light') : selectedTheme;

    // Update document classes
    document.documentElement.classList.remove('light', 'dark');
    document.documentElement.classList.add(actualTheme);

    // Update data-theme attribute
    document.documentElement.setAttribute('data-theme', actualTheme);

    // Update CSS custom properties for the theme
    updateCSSVariables(actualTheme);
  };

  const updateCSSVariables = (theme: 'light' | 'dark') => {
    if (theme === 'dark') {
      // Set dark theme variables
      document.documentElement.style.setProperty('--ifm-color-primary', '#4a90e2');
      document.documentElement.style.setProperty('--ifm-color-primary-dark', '#3a7bc8');
      document.documentElement.style.setProperty('--ifm-color-primary-darker', '#2a6bb8');
      document.documentElement.style.setProperty('--ifm-color-primary-darkest', '#1a5aa8');
      document.documentElement.style.setProperty('--ifm-color-primary-light', '#5aa0e6');
      document.documentElement.style.setProperty('--ifm-color-primary-lighter', '#6aa6ea');
      document.documentElement.style.setProperty('--ifm-color-primary-lightest', '#7aacee');
      document.documentElement.style.setProperty('--ifm-background-color', '#1a1a1a');
      document.documentElement.style.setProperty('--ifm-background-surface-color', '#222222');
      document.documentElement.style.setProperty('--ifm-font-color-base', '#e0e0e0');
      document.documentElement.style.setProperty('--ifm-code-background', '#2d2d2d');
      document.documentElement.style.setProperty('--ifm-code-color', '#f8f8f2');
    } else {
      // Reset to light theme variables
      document.documentElement.style.removeProperty('--ifm-color-primary');
      document.documentElement.style.removeProperty('--ifm-color-primary-dark');
      document.documentElement.style.removeProperty('--ifm-color-primary-darker');
      document.documentElement.style.removeProperty('--ifm-color-primary-darkest');
      document.documentElement.style.removeProperty('--ifm-color-primary-light');
      document.documentElement.style.removeProperty('--ifm-color-primary-lighter');
      document.documentElement.style.removeProperty('--ifm-color-primary-lightest');
      document.documentElement.style.removeProperty('--ifm-background-color');
      document.documentElement.style.removeProperty('--ifm-background-surface-color');
      document.documentElement.style.removeProperty('--ifm-font-color-base');
      document.documentElement.style.removeProperty('--ifm-code-background');
      document.documentElement.style.removeProperty('--ifm-code-color');
    }
  };

  const toggleTheme = () => {
    const newTheme = theme === 'light' ? 'dark' : theme === 'dark' ? 'auto' : 'light';
    setTheme(newTheme);
    localStorage.setItem('theme', newTheme);

    const systemTheme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    applyTheme(newTheme, systemTheme);
  };

  // Only render after mounting to avoid hydration issues
  if (!mounted) {
    return <div className={clsx(styles.container, className, styles.hidden)}></div>;
  }

  return (
    <div className={clsx(styles.container, className)}>
      <button
        className={clsx(
          styles.themeButton,
          theme === 'light' && styles.light,
          theme === 'dark' && styles.dark,
          theme === 'auto' && styles.auto
        )}
        onClick={toggleTheme}
        aria-label={`Switch to ${theme === 'light' ? 'dark' : theme === 'dark' ? 'auto' : 'light'} theme`}
        title={`Current theme: ${theme} (${theme === 'auto' ? 'system' : theme})`}
      >
        {theme === 'light' ? (
          <span className={styles.icon} aria-hidden="true">☀️</span>
        ) : theme === 'dark' ? (
          <span className={styles.icon} aria-hidden="true">🌙</span>
        ) : (
          <span className={styles.icon} aria-hidden="true">🔄</span>
        )}
        <span className={styles.themeLabel}>
          {theme === 'light' ? 'Light' : theme === 'dark' ? 'Dark' : 'Auto'}
        </span>
      </button>
    </div>
  );
}