import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Intro', // Sidebar label yahan se control hoga
    },
    {
      type: 'doc',
      id: 'module-1-ros2',
      label: 'Module 1: ROS 2',
    },
    {
      type: 'doc',
      id: 'module-2-gazebo',
      label: 'Module 2: Gazebo',
    },
    {
      type: 'doc',
      id: 'module-3-isaac',
      label: 'Module 3: NVIDIA Isaac',
    },
    {
      type: 'doc',
      id: 'module-4-vla',
      label: 'Module 4: VLA',
    },
    {
      type: 'doc',
      id: 'capstone',
      label: 'Capstone Project',
    },
  ],
};

export default sidebars;