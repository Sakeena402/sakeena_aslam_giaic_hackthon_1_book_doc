import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI & Humanoid Robotics',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Explore how artificial intelligence moves beyond screens into the real
        world. This book focuses on embodied intelligence, where AI systems
        perceive, reason, and act through humanoid robots in physical
        environments.
      </>
    ),
  },
  {
    title: 'From Simulation to Reality',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Learn how Digital Twins, physics simulation, and AI-powered perception
        enable safe and scalable robot development. Concepts progress from ROS 2
        foundations to high-fidelity simulation and autonomous navigation.
      </>
    ),
  },
  {
    title: 'Vision, Language, and Action',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Understand how modern AI models transform natural language and visual
        perception into real robot actions. The book culminates in building an
        autonomous humanoid capable of planning, navigation, and interaction.
      </>
    ),
  },
];

function Feature({ title, Svg, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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
