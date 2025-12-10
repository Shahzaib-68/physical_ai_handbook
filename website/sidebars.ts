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
      label: 'Introduction',
      items: ['01-intro/index'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Kinematics',
      items: ['02-kinematics/index'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Dynamics',
      items: ['03-dynamics/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Sensors',
      items: ['04-sensors/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Control',
      items: ['05-control/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Planning',
      items: ['06-planning/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Learning',
      items: ['07-learning/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Vision',
      items: ['08-vision/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Manipulation',
      items: ['09-manipulation/index'],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Humanoids',
      items: ['10-humanoids/index'],
      collapsed: true,
    },
  ],
};

export default sidebars;
