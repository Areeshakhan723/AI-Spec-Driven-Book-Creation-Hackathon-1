// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2-nervous-system/index',
        'module-1-ros2-nervous-system/chapter-1-intro-to-ros2',
        'module-1-ros2-nervous-system/chapter-2-communication-model',
        'module-1-ros2-nervous-system/chapter-3-urdf-structure',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin-gazebo-unity/index',
        'module-2-digital-twin-gazebo-unity/chapter-1-physics-simulation-gazebo',
        'module-2-digital-twin-gazebo-unity/chapter-2-digital-twins-unity',
        'module-2-digital-twin-gazebo-unity/chapter-3-sensor-simulation-validation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac-ai-brain/index',
        'module-3-isaac-ai-brain/chapter-1-isaac-sim-photorealistic',
        'module-3-isaac-ai-brain/chapter-2-isaac-ros-vslam-navigation',
        'module-3-isaac-ai-brain/chapter-3-nav2-path-planning-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla-integration/index',
        'module-4-vla-integration/chapter-1-vla-foundations',
        'module-4-vla-integration/chapter-2-voice-to-action-whisper',
        'module-4-vla-integration/chapter-3-cognitive-planning-llms',
        'module-4-vla-integration/chapter-4-capstone-autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorials/basic-ros2-node',
        'tutorials/urdf-tutorial',
        'tutorials/gazebo-simulation-tutorial',
        'tutorials/unity-digital-twin-tutorial',
        'tutorials/sensor-simulation-tutorial',
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/ros2-concepts',
        'reference/urdf-reference',
        'reference/gazebo-unity-sensors-reference',
      ],
    },
    'faq',
  ],
};

module.exports = sidebars;