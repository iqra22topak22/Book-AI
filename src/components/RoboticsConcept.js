import React from 'react';
import styles from './RoboticsConcept.module.css';

// Component to display robotics concepts with visual elements
export default function RoboticsConcept({title, children, image, difficulty}) {
  return (
    <div className={styles.conceptContainer}>
      <div className={styles.conceptHeader}>
        <h3 className={styles.conceptTitle}>{title}</h3>
        {difficulty && (
          <span className={`${styles.difficultyBadge} ${
            difficulty === 'Beginner' ? styles.beginner :
            difficulty === 'Intermediate' ? styles.intermediate :
            styles.advanced
          }`}>
            {difficulty}
          </span>
        )}
      </div>
      <div className={styles.conceptBody}>
        {image && <img src={image} alt={title} className={styles.conceptImage} />}
        <div className={styles.conceptContent}>
          {children}
        </div>
      </div>
    </div>
  );
}