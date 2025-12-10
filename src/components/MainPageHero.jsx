import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import NoSsr from './NoSsr';

// ---------------- Module Card Data ----------------
const moduleData = [
  { week: 1, title: "Introduction to ROS 2", description: "Fundamentals of ROS 2, nodes, topics, and services.", icon: "FaRobot", link: "/docs/course-modules/week-1" },
  { week: 2, title: "ROS 2 Navigation & Perception", description: "SLAM, navigation stacks, and sensor integration.", icon: "FaEye", link: "/docs/course-modules/week-2" },
  { week: 3, title: "Digital Twin with Gazebo", description: "Building robot models and simulating environments.", icon: "FaCubes", link: "/docs/course-modules/week-3" },
  { week: 4, title: "Advanced Simulation in Unity", description: "High-fidelity physics and realistic rendering.", icon: "FaUnity", link: "/docs/course-modules/week-4" },
  { week: 5, title: "NVIDIA Isaac Sim & ROS 2", description: "Integrating Isaac Sim with ROS 2 workflows.", icon: "FaMicrochip", link: "/docs/course-modules/week-5" },
  { week: 6, title: "AI with NVIDIA Jetson", description: "Deploying AI models on edge devices.", icon: "FaBrain", link: "/docs/course-modules/week-6" },
  { week: 7, title: "Robot Manipulation", description: "Inverse kinematics, motion planning, and grasping.", icon: "FaHandsHelping", link: "/docs/course-modules/week-7" },
  { week: 8, title: "Reinforcement Learning for Robotics", description: "Training robots with RL in simulated environments.", icon: "FaFlask", link: "/docs/course-modules/week-8" },
  { week: 9, title: "Vision-Language Models (VLMs)", description: "Understanding and generating human-like language for robots.", icon: "FaComments", link: "/docs/course-modules/week-9" },
  { week: 10, title: "Visual-Language-Action (VLA) Models", description: "Bridging vision, language, and action for complex tasks.", icon: "FaBook", link: "/docs/course-modules/week-10" },
  { week: 11, title: "Human-Robot Interaction", description: "Safe and intuitive collaboration with robots.", icon: "FaChalkboardTeacher", link: "/docs/course-modules/week-11" },
  { week: 12, title: "Project Development & Deployment", description: "Bringing your robotic ideas to life.", icon: "FaCode", link: "/docs/course-modules/week-12" },
  { week: 13, title: "Future of Physical AI", description: "Exploring advanced topics and emerging trends.", icon: "FaCogs", link: "/docs/course-modules/week-13" },
];


// ---------------- Content Cards ----------------
const contentCardsData = [
  {
    icon: "FaLightbulb",
    title: "Innovative Curriculum",
    description:
      "Our curriculum is designed around the latest advancements in robotics and Physical AI. Each module introduces concepts through practical, real-world applications. You will work on hands-on projects that strengthen your understanding and technical abilities. The content is continuously updated to match current industry standards. By the end, you will have a strong foundation built through experience, not theory alone."
  },
  {
    icon: "FaRocket",
    title: "Career Advancement",
    description:
      "This program equips you with skills that are highly valued across technology-driven industries. You will gain practical experience that makes you competitive in emerging job markets. The curriculum focuses on both technical depth and problem-solving capabilities. Employers actively seek professionals who can innovate using robotics and AI. Completing this course positions you strongly for growth opportunities in your career."
  },
  {
    icon: "FaHandsHelping",
    title: "Community Support",
    description:
      "You will become part of a thriving community of learners and professionals. The platform encourages peer-to-peer collaboration and shared problem solving. Instructors and mentors are accessible to guide you throughout your journey. Regular discussions and group activities help reinforce concepts and expand your network. This supportive environment ensures you are never learning alone."
  },
  {
    icon: "FaShieldAlt",
    title: "Expert Instructors",
    description:
      "Our instructors are industry veterans and academic leaders with deep expertise in Physical AI. They teach using real-world case studies and practical examples. Their guidance helps bridge the gap between theory and application. You will receive insights that come directly from years of professional experience. This level of mentorship ensures your learning is both accurate and industry-aligned."
  },
  {
    icon: "FaChalkboardTeacher",
    title: "Flexible Learning",
    description:
      "The program is designed to fit seamlessly into your schedule. You can learn at your own pace without compromising depth or quality. Each module includes structured lessons that are easy to follow. Additional resources are provided to support different learning styles. This flexibility ensures consistent progress whether you study daily or on weekends."
  },
  {
    icon: "FaGraduationCap",
    title: "Certification",
    description:
      "Upon completion, you will earn a recognized certificate validating your skills in Physical AI. This certification demonstrates your commitment to professional growth. It serves as a strong addition to your résumé or portfolio. Employers appreciate credentials backed by practical learning and real projects. Your certification reflects not just completion, but genuine expertise."
  }
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
            <Link className="button button--secondary button--lg" to="/docs/course-modules/">View Modules</Link>
          </div>
        </div>

      </div>

      {/* ----- Content Cards Section (Full-Width) ----- */}
      <section className="content-section">
        <h2 className="content-heading">What You Will Learn</h2>

        <div className="content-grid">
          {contentCardsData.map((card, index) => (
            <div key={index} className="content-item">
              <div className="content-icon">
                <NoSsr><IconLoader iconName={card.icon} /></NoSsr>
              </div>

              <div className="content-text">
                <h3>{card.title}</h3>
                <p>{card.description}</p>
              </div>

              {/* neon divider */}
              <div className="content-divider"></div>
            </div>
          ))}
        </div>
      </section>

    </>
  );
}
