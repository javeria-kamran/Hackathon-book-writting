import type {Config} from '@docusaurus/types';
import {themes as prismThemes} from 'prism-react-renderer';

const config: Config = {
  title: 'The Guide to Modern Agentic Development',
  tagline: 'Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics.example.com', // Updated URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'abdul-rehan7', // Updated GitHub org/user name.
  projectName: 'Hackathon-I-Physical-AI-Humanoid-Robotics', // Updated GitHub repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/abdul-rehan7/Hackathon-I-Physical-AI-Humanoid-Robotics/tree/main/', // Updated editUrl
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/abdul-rehan7/Hackathon-I-Physical-AI-Humanoid-Robotics/tree/main/', // Updated editUrl
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
      // Replace with your project's social card
      image: undefined, // Removed Docusaurus social card reference
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'The Guide to Modern Agentic Development',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          { to: '/docs/intro', label: 'Home', position: 'left' },
          { to: '/docs/course-modules', label: 'Start Learning', position: 'left' },
          { to: '/docs/why-physical-ai-matters', label: 'Why Physical AI', position: 'left' },
          { to: '/docs/learning-outcomes', label: 'Learning Outcomes', position: 'left' },
          { to: '/docs/hardware-requirements', label: 'Hardware Requirements', position: 'left' },
          { href: 'https://github.com/abdul-rehan7/Hackathon-I-Physical-AI-Humanoid-Robotics', label: 'GitHub', position: 'right' },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord', // Keeping Discord link if applicable
                href: 'https://discordapp.com/invite/docusaurus', // Placeholder - user may want to change
              },
              {
                label: 'Twitter', // Changed from X to Twitter for common usage
                href: 'https://twitter.com/docusaurus', // Placeholder - user may want to change
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/abdul-rehan7/Hackathon-I-Physical-AI-Humanoid-Robotics', // Updated GitHub link
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`, // Updated copyright
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    },
};

export default config;