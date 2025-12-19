import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to building embodied AI systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://HassaanGhayas.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-robotics-textbook/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'HassaanGhayas', // Usually your GitHub org/user name.
  projectName: 'physical-ai-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    format: 'detect',
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  customFields: {
    ragApiUrl: process.env.REACT_APP_API_URL || 'https://iterateee-physical-ai-robotics-backend.hf.space',
    chatbotEnabled: process.env.REACT_APP_CHATBOT_ENABLED !== 'false', // Enabled by default
    maxRetries: parseInt(process.env.REACT_APP_MAX_RETRIES || '3', 10),
    timeoutMs: parseInt(process.env.REACT_APP_TIMEOUT_MS || '30000', 10), // 30s for production
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Edit URL for the book repo
          editUrl:
            'https://github.com/HassaanGhayas/physical-ai-robotics-textbook/tree/main/',
          remarkPlugins: [],
        },
        blog: false, // Disable blog - not needed for book
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    './plugins/tailwind-config.cjs',
    './plugins/alias-plugin.cjs',
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/HassaanGhayas/physical-ai-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Chapters',
          items: [
            {
              label: 'Hardware Requirements',
              to: '/docs/book/hardware-requirements',
            },
            {
              label: 'Introduction',
              to: '/docs/book/introduction',
            },
            {
              label: 'Robotic Nervous System',
              to: '/docs/book/robotic-nervous-system',
            },
            {
              label: 'Digital Twin',
              to: '/docs/book/digital-twin',
            },
            {
              label: 'AI-Robot Brain',
              to: '/docs/book/ai-robot-brain',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/book/vision-language-action',
            },
            {
              label: 'Assessments',
              to: '/docs/book/assessments',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/HassaanGhayas/physical-ai-robotics-textbook',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/physical-ai',
            },
            {
              label: 'Documentation',
              href: 'https://your-org.github.io/physical-ai-book',
            },
          ],
        },
        {
          title: 'Legal',
          items: [
            {
              label: 'Privacy',
              to: '/docs/privacy',
            },
            {
              label: 'Terms',
              to: '/docs/terms',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'latex'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
