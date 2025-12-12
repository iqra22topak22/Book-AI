import React from 'react';
import clsx from 'clsx';
import styles from './CourseCard.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Robotics',
    Svg: require('@site/static/img/robot-arm.svg').default,
    description: (
      <>
        Learn about embodied intelligence, humanoid robotics, and AI integration
        using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.
      </>
    ),
  },
  {
    title: 'Simulation & Reality',
    Svg: require('@site/static/img/simulation.svg').default,
    description: (
      <>
        Experience both digital twin environments with Gazebo/Unity and
        real hardware with Unitree and Hiwonder robots.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/hands-on.svg').default,
    description: (
      <>
        Every concept includes practical, executable examples and projects
        that reinforce theoretical concepts with real implementations.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
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