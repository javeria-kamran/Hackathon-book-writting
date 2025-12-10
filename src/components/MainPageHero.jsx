import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import NoSsr from './NoSsr';

// ---------------- Module Card Data ----------------
const moduleData = [
  { week: 1, title: "Introduction to ROS 2", description: "Fundamentals of ROS 2, nodes, topics, and services.", icon: "FaRobot", link: "/docs/course-modules/week-1/index" },
  { week: 2, title: "ROS 2 Navigation & Perception", description: "SLAM, navigation stacks, and sensor integration.", icon: "FaEye", link: "/docs/course-modules/week-2/index" },
  { week: 3, title: "Digital Twin with Gazebo", description: "Building robot models and simulating environments.", icon: "FaCubes", link: "/docs/course-modules/week-3/index" },
  { week: 4, title: "Advanced Simulation in Unity", description: "High-fidelity physics and realistic rendering.", icon: "FaUnity", link: "/docs/course-modules/week-4/index" },
  { week: 5, title: "NVIDIA Isaac Sim & ROS 2", description: "Integrating Isaac Sim with ROS 2 workflows.", icon: "FaMicrochip", link: "/docs/course-modules/week-5/index" },
  { week: 6, title: "AI with NVIDIA Jetson", description: "Deploying AI models on edge devices.", icon: "FaBrain", link: "/docs/course-modules/week-6/index" },
  { week: 7, title: "Robot Manipulation", description: "Inverse kinematics, motion planning, and grasping.", icon: "FaHandsHelping", link: "/docs/course-modules/week-7/index" },
  { week: 8, title: "Reinforcement Learning for Robotics", description: "Training robots with RL in simulated environments.", icon: "FaFlask", link: "/docs/course-modules/week-8/index" },
  { week: 9, title: "Vision-Language Models (VLMs)", description: "Understanding and generating human-like language for robots.", icon: "FaComments", link: "/docs/course-modules/week-9/index" },
  { week: 10, title: "Visual-Language-Action (VLA) Models", description: "Bridging vision, language, and action for complex tasks.", icon: "FaBook", link: "/docs/course-modules/week-10/index" },
  { week: 11, title: "Human-Robot Interaction", description: "Safe and intuitive collaboration with robots.", icon: "FaChalkboardTeacher", link: "/docs/course-modules/week-11/index" },
  { week: 12, title: "Project Development & Deployment", description: "Bringing your robotic ideas to life.", icon: "FaCode", link: "/docs/course-modules/week-12/index" },
  { week: 13, title: "Future of Physical AI", description: "Exploring advanced topics and emerging trends.", icon: "FaCogs", link: "/docs/course-modules/week-13/index" },
];

// ---------------- Content Cards ----------------
const contentCardsData = [
  { icon: "FaLightbulb", title: "Innovative Curriculum", description: "Cutting-edge topics, project-based learning, and hands-on experience." },
  { icon: "FaRocket", title: "Career Advancement", description: "Gain skills highly sought after in the rapidly growing field of robotics and AI." },
  { icon: "FaHandsHelping", title: "Community Support", description: "Join a vibrant community of learners and experts, fostering collaboration and growth." },
  { icon: "FaShieldAlt", title: "Expert Instructors", description: "Learn from industry leaders and academic experts with deep knowledge in Physical AI." },
  { icon: "FaChalkboardTeacher", title: "Flexible Learning", description: "Self-paced modules and comprehensive resources to fit your schedule." },
  { icon: "FaGraduationCap", title: "Certification", description: "Earn a certificate of completion to validate your expertise in Physical AI." },
];

// ---------------- Dynamic Icon Loader ----------------
function IconLoader({ iconName, ...props }) {
  const [IconComponent, setIconComponent] = useState(null);
  useEffect(() => {
    if (iconName) {
      import('react-icons/fa').then(module => {
        if (module[iconName]) setIconComponent(() => module[iconName]);
        else setIconComponent(() => module.FaRobot);
      });
    }
  }, [iconName]);
  return IconComponent ? <IconComponent {...props} /> : null;
}

export default function MainPageHero() {
  const sizeClasses = [
    "size-a", "size-b", "size-c",
    "size-d", "size-b", "size-c",
    "size-e", "size-b", "size-c",
    "size-d", "size-c", "size-b",
    "size-a"
  ];

  return (
    <>
      <div className="main-page-hero">

        {/* ----- Masonry Grid ----- */}
        {/* Cards Grid (4 columns x 3 rows, equal sizes) */}
        <div className="hero-modules-grid">
          {moduleData.slice(0, 12).map((module) => (
            <Link key={module.week} to={module.link} className="hero-module-card">
              <div className="card-content">
                <div className="card-icon-wrapper">
                  <NoSsr><IconLoader iconName={module.icon} className="card-icon" /></NoSsr>
                </div>
                <h3 className="card-title">{module.title}</h3>
                <p className="card-description">{module.description}</p>
                <div className="card-button-wrapper">
                  <button className="button button--outline button--sm">Learn More</button>
                </div>
              </div>
            </Link>
          ))}
        </div>


        {/* ----- Hero Text ----- */}
        <div className="hero-content-wrapper">
          <h1 className="hero-title">From Code to Consciousness</h1>
          <p className="hero-subtitle">
            A 13-week journey to build autonomous humanoid robots with ROS 2, NVIDIA Isaac, and Vision-Language-Action models.
          </p>
          <div className="hero-buttons">
            <Link className="button button--primary button--lg" to="/docs/intro">Get Started</Link>
            <Link className="button button--secondary button--lg" to="/docs/course-modules/index">View Modules</Link>
          </div>
        </div>

      </div>

      {/* ----- Content Cards Section (Full-Width) ----- */}
      <section className="content-cards-section">
        <div className="content-cards-grid">
          {contentCardsData.map((card, index) => (
            <div key={index} className="content-card">
              <div className="content-card-icon">
                <NoSsr><IconLoader iconName={card.icon} /></NoSsr>
              </div>
              <h3>{card.title}</h3>
              <p>{card.description}</p>
            </div>
          ))}
        </div>
      </section>
    </>
  );
}
