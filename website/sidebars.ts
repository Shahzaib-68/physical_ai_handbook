import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      items: ['intro/index'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1-1',
        'module-1/chapter-1-2',
        'module-1/chapter-1-3',
        'module-1/chapter-1-4',
        'module-1/chapter-1-5',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/chapter-2-1',
        'module-2/chapter-2-2',
        'module-2/chapter-2-3',
        'module-2/chapter-2-4',
        'module-2/chapter-2-5',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/chapter-3-1',
        'module-3/chapter-3-2',
        'module-3/chapter-3-3',
        'module-3/chapter-3-4',
        'module-3/chapter-3-5',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/chapter-4-1',
        'module-4/chapter-4-2',
        'module-4/chapter-4-3',
        'module-4/chapter-4-4',
        'module-4/chapter-4-5',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'hardware-requirements',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;
