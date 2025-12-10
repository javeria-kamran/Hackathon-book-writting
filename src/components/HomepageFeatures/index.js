import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';

const FeatureList = [
  {
    emoji: '🤖',
    title: 'Agentic Development',
    description: 'Build goal-oriented autonomous systems that plan and execute actions.',
  },
  {
    emoji: '🎮',
    title: 'High-Fidelity Simulation',
    description: 'Leverage NVIDIA Isaac Sim for physically accurate virtual environments.',
  },
  {
    emoji: '🧠',
    title: 'AI & Perception',
    description: 'Integrate advanced computer vision and sensor fusion for robust environmental understanding.',
  },
  {
    emoji: '🦾',
    title: 'Robotics Control',
    description: 'Master ROS 2, kinematics, and MoveIt 2 for precise robot manipulation.',
  },
  {
    emoji: '🗣️',
    title: 'Natural Language Interaction',
    description: 'Enable voice commands and LLM-based task planning for intuitive human-robot communication.',
  },
  {
    emoji: '⚙️',
    title: 'Reproducible Environments',
    description: 'Utilize Docker for consistent and portable development setups.',
  },
];

function Feature({emoji, title, description}) {
  return (
    <div className="feature-card">
      <div className="feature-icon" aria-hidden="true">{emoji}</div>
      <div className="feature-body">
        <Heading as="h3" className="feature-title">{title}</Heading>
        <p className="feature-desc">{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className="features-section" aria-labelledby="what-you-will-learn">
      <div className="features-container">
        <h2 id="what-you-will-learn" className="features-heading">What you will learn</h2>
        <div className="features-grid">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
        <div className="features-cta">
          <Link className="button button--primary" to="/docs/learning-outcomes">Learning Outcomes</Link>
          <Link className="button button--secondary" to="/docs/hardware-requirements">Hardware Requirements</Link>
        </div>
      </div>
    </section>
  );
}
