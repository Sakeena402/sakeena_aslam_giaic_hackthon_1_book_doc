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
  // Main book sidebar with book introduction and modules
  bookSidebar: [
    {
      type: 'category',
      label: 'Book Introduction',
      items: ['book/intro'],
    },
    {
      type: 'category',
      label: 'Course Modules',
      items: [
        {
          type: 'category',
          label: 'Module 01: ROS 2 - The Robotic Nervous System',
          items: [
            'modules/ros2/index',
            'modules/ros2/chapter1',
            'modules/ros2/chapter2',
            'modules/ros2/chapter3'
          ],
        },
        // Additional modules can be added here in the future
      ],
    },
  ],

  // Keep the ROS2 module sidebar for reference (can be removed later if desired)
  ros2ModuleSidebar: [
    {
      type: 'category',
      label: 'ROS 2 - The Robotic Nervous System',
      items: [
        'modules/ros2/index',
        'modules/ros2/chapter1',
        'modules/ros2/chapter2',
        'modules/ros2/chapter3'
      ],
    },
  ],
};

export default sidebars;
