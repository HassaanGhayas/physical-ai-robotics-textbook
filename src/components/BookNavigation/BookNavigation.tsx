import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './BookNavigation.module.css';

type NavigationItem = {
  id: string;
  title: string;
  url: string;
  level: number;
  isCompleted?: boolean;
};

type BookNavigationProps = {
  title: string;
  items: NavigationItem[];
  currentId?: string;
  onNavigate?: (item: NavigationItem) => void;
  showProgress?: boolean;
  showBookmarks?: boolean;
};

export default function BookNavigation({
  title,
  items,
  currentId,
  onNavigate,
  showProgress = true,
  showBookmarks = true
}: BookNavigationProps): JSX.Element {
  const [bookmarkedItems, setBookmarkedItems] = useState<Set<string>>(new Set());
  const [progress, setProgress] = useState<Record<string, boolean>>({});

  useEffect(() => {
    // Load bookmarks from localStorage
    const savedBookmarks = localStorage.getItem('bookNavigationBookmarks');
    if (savedBookmarks) {
      try {
        setBookmarkedItems(new Set(JSON.parse(savedBookmarks)));
      } catch (e) {
        console.error('Error loading bookmarks:', e);
      }
    }

    // Load progress from localStorage
    const savedProgress = localStorage.getItem('bookNavigationProgress');
    if (savedProgress) {
      try {
        setProgress(JSON.parse(savedProgress));
      } catch (e) {
        console.error('Error loading progress:', e);
      }
    }
  }, []);

  const toggleBookmark = (itemId: string) => {
    const newBookmarks = new Set(bookmarkedItems);
    if (newBookmarks.has(itemId)) {
      newBookmarks.delete(itemId);
    } else {
      newBookmarks.add(itemId);
    }
    setBookmarkedItems(newBookmarks);
    localStorage.setItem('bookNavigationBookmarks', JSON.stringify(Array.from(newBookmarks)));
  };

  const markAsCompleted = (itemId: string) => {
    const newProgress = { ...progress, [itemId]: true };
    setProgress(newProgress);
    localStorage.setItem('bookNavigationProgress', JSON.stringify(newProgress));
  };

  const handleItemClick = (item: NavigationItem) => {
    if (onNavigate) {
      onNavigate(item);
    }
    // Mark current item as completed
    markAsCompleted(item.id);
  };

  // Calculate progress percentage
  const completedCount = Object.values(progress).filter(Boolean).length;
  const totalCount = items.length;
  const progressPercentage = totalCount > 0 ? Math.round((completedCount / totalCount) * 100) : 0;

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3 className={styles.title}>{title}</h3>
        {showProgress && (
          <div className={styles.progressContainer}>
            <div className={styles.progressBar}>
              <div
                className={styles.progressFill}
                style={{ width: `${progressPercentage}%` }}
              />
            </div>
            <span className={styles.progressText}>{progressPercentage}%</span>
          </div>
        )}
      </div>

      <nav className={styles.navigation}>
        <ul className={styles.navigationList}>
          {items.map((item) => (
            <li
              key={item.id}
              className={clsx(
                styles.navigationItem,
                item.level > 1 && styles.nested,
                currentId === item.id && styles.current,
                progress[item.id] && styles.completed
              )}
              style={{ paddingLeft: `${item.level * 1}rem` }}
            >
              <a
                href={item.url}
                className={styles.navigationLink}
                onClick={(e) => {
                  e.preventDefault();
                  handleItemClick(item);
                }}
              >
                <span className={styles.itemTitle}>{item.title}</span>
                {progress[item.id] && (
                  <span className={styles.completedIcon}>✓</span>
                )}
              </a>

              {showBookmarks && (
                <button
                  className={clsx(
                    styles.bookmarkButton,
                    bookmarkedItems.has(item.id) && styles.bookmarked
                  )}
                  onClick={(e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    toggleBookmark(item.id);
                  }}
                  title={bookmarkedItems.has(item.id) ? 'Remove bookmark' : 'Bookmark this item'}
                >
                  {bookmarkedItems.has(item.id) ? '★' : '☆'}
                </button>
              )}
            </li>
          ))}
        </ul>
      </nav>

      {showBookmarks && (
        <div className={styles.bookmarksSection}>
          <h4 className={styles.bookmarksTitle}>Bookmarks</h4>
          <ul className={styles.bookmarksList}>
            {items
              .filter(item => bookmarkedItems.has(item.id))
              .map(item => (
                <li key={item.id} className={styles.bookmarkItem}>
                  <a
                    href={item.url}
                    className={styles.bookmarkLink}
                    onClick={(e) => {
                      e.preventDefault();
                      handleItemClick(item);
                    }}
                  >
                    {item.title}
                  </a>
                </li>
              ))}
          </ul>
          {bookmarkedItems.size === 0 && (
            <p className={styles.noBookmarks}>No bookmarks yet</p>
          )}
        </div>
      )}
    </div>
  );
}