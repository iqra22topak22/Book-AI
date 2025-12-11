// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros-nervous-system/index',
        'module-1-ros-nervous-system/learning-outcomes',
        {
          type: 'category',
          label: 'Lessons',
          items: [
            'module-1-ros-nervous-system/lessons/lesson-1',
          ]
        },
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-1-ros-nervous-system/labs/lab-1',
          ]
        },
        {
          type: 'category',
          label: 'Deliverables',
          items: [
            'module-1-ros-nervous-system/deliverables/deliverable-1',
          ]
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/learning-outcomes',
        {
          type: 'category',
          label: 'Lessons',
          items: [
            'module-2-digital-twin/lessons/lesson-1',
          ]
        },
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-2-digital-twin/labs/lab-1',
          ]
        },
        {
          type: 'category',
          label: 'Deliverables',
          items: [
            'module-2-digital-twin/deliverables/deliverable-1',
          ]
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Brain',
      items: [
        'module-3-nvidia-isaac-brain/index',
        'module-3-nvidia-isaac-brain/learning-outcomes',
        {
          type: 'category',
          label: 'Lessons',
          items: [
            'module-3-nvidia-isaac-brain/lessons/lesson-1',
          ]
        },
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-3-nvidia-isaac-brain/labs/lab-1',
          ]
        },
        {
          type: 'category',
          label: 'Deliverables',
          items: [
            'module-3-nvidia-isaac-brain/deliverables/deliverable-1',
          ]
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vision-language-action/index',
        'module-4-vision-language-action/learning-outcomes',
        {
          type: 'category',
          label: 'Lessons',
          items: [
            'module-4-vision-language-action/lessons/lesson-1',
          ]
        },
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-4-vision-language-action/labs/lab-1',
          ]
        },
        {
          type: 'category',
          label: 'Deliverables',
          items: [
            'module-4-vision-language-action/deliverables/deliverable-1',
          ]
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorials/first-ros2-package',
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/ros-isaac-components',
      ],
    },
    {
      type: 'category',
      label: 'Guides',
      items: [
        'guides/capstone-guide',
      ],
    },
    {
      type: 'category',
      label: 'Instructor Resources',
      items: [
        'instructors/overview',
      ],
    },
    'glossary',
    'timeline',
  ],
};

module.exports = sidebars;