import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Easy to Use',
    imageUrl: '/img/easy-to-use.png',
    description: (
      <>
        Docusaurus was designed from the ground up to be easily installed and used to get your website up and running quickly.
      </>
    ),
  },
  {
    title: 'Focus on What Matters',
    imageUrl: '/img/focus-matters.png',
    description: (
      <>
        Docusaurus lets you focus on your docs, and we'll do the chores. Go ahead and move your docs into the docs directory.
      </>
    ),
  },
  {
    title: 'Powered by React',
    imageUrl: '/img/powered-by-react.png',
    description: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can be extended while reusing the same header and footer.
      </>
    ),
  },
];

function Feature({Svg, title, description, imageUrl}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        {imageUrl && (
          <div className="margin-bottom--lg">
            <img src={imageUrl} alt={title} className={styles.featureImage} />
          </div>
        )}
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}