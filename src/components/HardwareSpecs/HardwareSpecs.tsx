import React from 'react';
import clsx from 'clsx';
import styles from './HardwareSpecs.module.css';

type HardwareSpecProps = {
  name: string;
  category: string;
  specs: {
    cpu?: string;
    gpu?: string;
    memory?: string;
    storage?: string;
    [key: string]: string | undefined; // Allow additional spec properties
  };
  cost: {
    price: string;
    reasoning: string;
  };
  pros: string[];
  cons: string[];
};

export default function HardwareSpecs({
  name,
  category,
  specs,
  cost,
  pros,
  cons
}: HardwareSpecProps): JSX.Element {
  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <div className={styles.header}>
          <h3 className={styles.name}>{name}</h3>
          <span className={styles.category}>{category}</span>
        </div>

        <div className={styles.specSection}>
          {Object.entries(specs).map(([key, value]) => (
            value && (
              <div key={key} className={styles.specRow}>
                <div className={styles.specCol}>
                  <h4>{key.charAt(0).toUpperCase() + key.slice(1).replace(/([A-Z])/g, ' $1')}</h4>
                  <p>{value}</p>
                </div>
              </div>
            )
          ))}
        </div>

        <div className={styles.costRow}>
          <h4>Estimated Cost</h4>
          <p>{cost.price}</p>
        </div>

        <div className={styles.reasoningRow}>
          <h4>Reasoning</h4>
          <p>{cost.reasoning}</p>
        </div>

        <div className={styles.proConRow}>
          <div className={styles.proConCol}>
            <h4>Pros</h4>
            <ul>
              {pros.map((pro, idx) => (
                <li key={idx}>{pro}</li>
              ))}
            </ul>
          </div>
          <div className={styles.proConCol}>
            <h4>Cons</h4>
            <ul>
              {cons.map((con, idx) => (
                <li key={idx}>{con}</li>
              ))}
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}