import React from 'react';
import clsx from 'clsx';
import styles from './HardwareSpecs.module.css';

type HardwareSpecItem = {
  name: string;
  category: string;
  minSpec: string;
  idealSpec: string;
  cost: string;
  pros: string[];
  cons: string[];
  reasoning: string;
};

export default function HardwareSpecs({specs}: {specs: HardwareSpecItem[]}): JSX.Element {
  return (
    <div className={styles.container}>
      <div className={styles.grid}>
        {specs.map((spec, index) => (
          <div key={index} className={styles.card}>
            <div className={styles.header}>
              <h3 className={styles.name}>{spec.name}</h3>
              <span className={styles.category}>{spec.category}</span>
            </div>

            <div className={styles.specRow}>
              <div className={styles.specCol}>
                <h4>Minimum</h4>
                <p>{spec.minSpec}</p>
              </div>
              <div className={styles.specCol}>
                <h4>Ideal</h4>
                <p>{spec.idealSpec}</p>
              </div>
            </div>

            <div className={styles.costRow}>
              <h4>Estimated Cost</h4>
              <p>{spec.cost}</p>
            </div>

            <div className={styles.reasoningRow}>
              <h4>Reasoning</h4>
              <p>{spec.reasoning}</p>
            </div>

            <div className={styles.proConRow}>
              <div className={styles.proConCol}>
                <h4>Pros</h4>
                <ul>
                  {spec.pros.map((pro, idx) => (
                    <li key={idx}>{pro}</li>
                  ))}
                </ul>
              </div>
              <div className={styles.proConCol}>
                <h4>Cons</h4>
                <ul>
                  {spec.cons.map((con, idx) => (
                    <li key={idx}>{con}</li>
                  ))}
                </ul>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}