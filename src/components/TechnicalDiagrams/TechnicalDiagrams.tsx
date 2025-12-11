import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './TechnicalDiagrams.module.css';

type TechnicalDiagramProps = {
  title: string;
  description: string;
  imageUrl: string;
  caption: string;
  altText: string;
  interactive?: boolean;
  zoomable?: boolean;
};

export default function TechnicalDiagrams({
  title,
  description,
  imageUrl,
  caption,
  altText,
  interactive = true,
  zoomable = true
}: TechnicalDiagramProps): JSX.Element {
  const [zoomed, setZoomed] = useState(false);

  return (
    <div className={clsx(styles.container, zoomable && styles.zoomable)}>
      <div className={styles.diagramHeader}>
        <h3>{title}</h3>
      </div>
      <div className={styles.diagramContent}>
        <div className={styles.imageContainer}>
          <img
            src={imageUrl}
            alt={altText}
            className={clsx(
              styles.diagramImage,
              zoomed && styles.zoomedImage
            )}
            onClick={() => zoomable && setZoomed(!zoomed)}
          />
          {interactive && (
            <div className={styles.overlayControls}>
              <button
                className={styles.zoomButton}
                onClick={(e) => {
                  e.stopPropagation();
                  setZoomed(!zoomed);
                }}
              >
                {zoomed ? 'Shrink' : 'Zoom'}
              </button>
            </div>
          )}
        </div>
        <div className={styles.captionContainer}>
          <p className={styles.description}>{description}</p>
          <p className={styles.caption}><em>{caption}</em></p>
        </div>
      </div>
    </div>
  );
}