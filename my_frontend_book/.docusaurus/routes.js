import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '2ad'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '979'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '4c5'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'cdc'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', 'f62'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '3ce'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '674'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'c24'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '077'),
        routes: [
          {
            path: '/docs/tags',
            component: ComponentCreator('/docs/tags', '0cc'),
            exact: true
          },
          {
            path: '/docs/tags/communication',
            component: ComponentCreator('/docs/tags/communication', '65f'),
            exact: true
          },
          {
            path: '/docs/tags/concepts',
            component: ComponentCreator('/docs/tags/concepts', 'd48'),
            exact: true
          },
          {
            path: '/docs/tags/faq',
            component: ComponentCreator('/docs/tags/faq', '3c2'),
            exact: true
          },
          {
            path: '/docs/tags/fundamentals',
            component: ComponentCreator('/docs/tags/fundamentals', 'f89'),
            exact: true
          },
          {
            path: '/docs/tags/humanoid-robotics',
            component: ComponentCreator('/docs/tags/humanoid-robotics', '9b4'),
            exact: true
          },
          {
            path: '/docs/tags/middleware',
            component: ComponentCreator('/docs/tags/middleware', 'd85'),
            exact: true
          },
          {
            path: '/docs/tags/node',
            component: ComponentCreator('/docs/tags/node', 'c2b'),
            exact: true
          },
          {
            path: '/docs/tags/nodes',
            component: ComponentCreator('/docs/tags/nodes', 'ea0'),
            exact: true
          },
          {
            path: '/docs/tags/python',
            component: ComponentCreator('/docs/tags/python', 'd96'),
            exact: true
          },
          {
            path: '/docs/tags/rclpy',
            component: ComponentCreator('/docs/tags/rclpy', '72b'),
            exact: true
          },
          {
            path: '/docs/tags/reference',
            component: ComponentCreator('/docs/tags/reference', '8ce'),
            exact: true
          },
          {
            path: '/docs/tags/robot-description',
            component: ComponentCreator('/docs/tags/robot-description', '7e8'),
            exact: true
          },
          {
            path: '/docs/tags/ros-2',
            component: ComponentCreator('/docs/tags/ros-2', '51a'),
            exact: true
          },
          {
            path: '/docs/tags/services',
            component: ComponentCreator('/docs/tags/services', '2d2'),
            exact: true
          },
          {
            path: '/docs/tags/simulation',
            component: ComponentCreator('/docs/tags/simulation', 'db8'),
            exact: true
          },
          {
            path: '/docs/tags/terminology',
            component: ComponentCreator('/docs/tags/terminology', '5fb'),
            exact: true
          },
          {
            path: '/docs/tags/topics',
            component: ComponentCreator('/docs/tags/topics', '5c2'),
            exact: true
          },
          {
            path: '/docs/tags/troubleshooting',
            component: ComponentCreator('/docs/tags/troubleshooting', 'e48'),
            exact: true
          },
          {
            path: '/docs/tags/tutorial',
            component: ComponentCreator('/docs/tags/tutorial', 'c37'),
            exact: true
          },
          {
            path: '/docs/tags/urdf',
            component: ComponentCreator('/docs/tags/urdf', '787'),
            exact: true
          },
          {
            path: '/docs/tags/xml',
            component: ComponentCreator('/docs/tags/xml', 'e2d'),
            exact: true
          },
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'fe7'),
            routes: [
              {
                path: '/docs/faq',
                component: ComponentCreator('/docs/faq', 'e79'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2-nervous-system/',
                component: ComponentCreator('/docs/module-1-ros2-nervous-system/', '94d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2-nervous-system/chapter-1-intro-to-ros2',
                component: ComponentCreator('/docs/module-1-ros2-nervous-system/chapter-1-intro-to-ros2', '1f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2-nervous-system/chapter-2-communication-model',
                component: ComponentCreator('/docs/module-1-ros2-nervous-system/chapter-2-communication-model', '127'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2-nervous-system/chapter-3-urdf-structure',
                component: ComponentCreator('/docs/module-1-ros2-nervous-system/chapter-3-urdf-structure', '651'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin-gazebo-unity/',
                component: ComponentCreator('/docs/module-2-digital-twin-gazebo-unity/', '4d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin-gazebo-unity/chapter-1-physics-simulation-gazebo',
                component: ComponentCreator('/docs/module-2-digital-twin-gazebo-unity/chapter-1-physics-simulation-gazebo', '61a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin-gazebo-unity/chapter-2-digital-twins-unity',
                component: ComponentCreator('/docs/module-2-digital-twin-gazebo-unity/chapter-2-digital-twins-unity', '3a3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin-gazebo-unity/chapter-3-sensor-simulation-validation',
                component: ComponentCreator('/docs/module-2-digital-twin-gazebo-unity/chapter-3-sensor-simulation-validation', 'ee8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac-ai-brain/',
                component: ComponentCreator('/docs/module-3-isaac-ai-brain/', '79e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac-ai-brain/chapter-1-isaac-sim-photorealistic',
                component: ComponentCreator('/docs/module-3-isaac-ai-brain/chapter-1-isaac-sim-photorealistic', '7a1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac-ai-brain/chapter-2-isaac-ros-vslam-navigation',
                component: ComponentCreator('/docs/module-3-isaac-ai-brain/chapter-2-isaac-ros-vslam-navigation', 'c77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac-ai-brain/chapter-3-nav2-path-planning-humanoid',
                component: ComponentCreator('/docs/module-3-isaac-ai-brain/chapter-3-nav2-path-planning-humanoid', '975'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-integration/',
                component: ComponentCreator('/docs/module-4-vla-integration/', 'd67'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-integration/chapter-1-vla-foundations',
                component: ComponentCreator('/docs/module-4-vla-integration/chapter-1-vla-foundations', '6c8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-integration/chapter-2-voice-to-action-whisper',
                component: ComponentCreator('/docs/module-4-vla-integration/chapter-2-voice-to-action-whisper', '88d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-integration/chapter-3-cognitive-planning-llms',
                component: ComponentCreator('/docs/module-4-vla-integration/chapter-3-cognitive-planning-llms', '28d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-integration/chapter-4-capstone-autonomous-humanoid',
                component: ComponentCreator('/docs/module-4-vla-integration/chapter-4-capstone-autonomous-humanoid', 'c76'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/reference/gazebo-unity-sensors-reference',
                component: ComponentCreator('/docs/reference/gazebo-unity-sensors-reference', '6c3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/reference/ros2-concepts',
                component: ComponentCreator('/docs/reference/ros2-concepts', '033'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/reference/urdf-reference',
                component: ComponentCreator('/docs/reference/urdf-reference', '1df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/basic-ros2-node',
                component: ComponentCreator('/docs/tutorials/basic-ros2-node', '3de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/gazebo-simulation-tutorial',
                component: ComponentCreator('/docs/tutorials/gazebo-simulation-tutorial', 'cc0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/sensor-simulation-tutorial',
                component: ComponentCreator('/docs/tutorials/sensor-simulation-tutorial', '242'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/unity-digital-twin-tutorial',
                component: ComponentCreator('/docs/tutorials/unity-digital-twin-tutorial', 'e09'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/urdf-tutorial',
                component: ComponentCreator('/docs/tutorials/urdf-tutorial', '30b'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '0ea'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
