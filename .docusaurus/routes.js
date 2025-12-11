import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-course/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug', 'ce6'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/config', '7b9'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/content', '300'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/globalData', 'eba'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/metadata', '4fe'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/registry', '6a7'),
    exact: true
  },
  {
    path: '/physical-ai-course/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-course/__docusaurus/debug/routes', '77d'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog',
    component: ComponentCreator('/physical-ai-course/blog', 'fc1'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/archive',
    component: ComponentCreator('/physical-ai-course/blog/archive', 'a31'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/authors',
    component: ComponentCreator('/physical-ai-course/blog/authors', '1ba'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/first-blog-post',
    component: ComponentCreator('/physical-ai-course/blog/first-blog-post', '94e'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/iqra-author-introduction',
    component: ComponentCreator('/physical-ai-course/blog/iqra-author-introduction', '60b'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/long-blog-post',
    component: ComponentCreator('/physical-ai-course/blog/long-blog-post', '264'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/mdx-blog-post',
    component: ComponentCreator('/physical-ai-course/blog/mdx-blog-post', 'daf'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags',
    component: ComponentCreator('/physical-ai-course/blog/tags', '400'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/ai',
    component: ComponentCreator('/physical-ai-course/blog/tags/ai', 'e53'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/author',
    component: ComponentCreator('/physical-ai-course/blog/tags/author', 'b90'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/beginner',
    component: ComponentCreator('/physical-ai-course/blog/tags/beginner', 'dea'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/docusaurus',
    component: ComponentCreator('/physical-ai-course/blog/tags/docusaurus', 'd75'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/hello',
    component: ComponentCreator('/physical-ai-course/blog/tags/hello', '2a5'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/hola',
    component: ComponentCreator('/physical-ai-course/blog/tags/hola', 'f7c'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/introduction',
    component: ComponentCreator('/physical-ai-course/blog/tags/introduction', '768'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/journey',
    component: ComponentCreator('/physical-ai-course/blog/tags/journey', '2f6'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/programming',
    component: ComponentCreator('/physical-ai-course/blog/tags/programming', '8ff'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/robotics',
    component: ComponentCreator('/physical-ai-course/blog/tags/robotics', '402'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/tags/web-development',
    component: ComponentCreator('/physical-ai-course/blog/tags/web-development', '424'),
    exact: true
  },
  {
    path: '/physical-ai-course/blog/welcome-anusha',
    component: ComponentCreator('/physical-ai-course/blog/welcome-anusha', 'e41'),
    exact: true
  },
  {
    path: '/physical-ai-course/markdown-page',
    component: ComponentCreator('/physical-ai-course/markdown-page', '59c'),
    exact: true
  },
  {
    path: '/physical-ai-course/docs',
    component: ComponentCreator('/physical-ai-course/docs', '14f'),
    routes: [
      {
        path: '/physical-ai-course/docs',
        component: ComponentCreator('/physical-ai-course/docs', 'c88'),
        routes: [
          {
            path: '/physical-ai-course/docs',
            component: ComponentCreator('/physical-ai-course/docs', 'df8'),
            routes: [
              {
                path: '/physical-ai-course/docs/capstone-project',
                component: ComponentCreator('/physical-ai-course/docs/capstone-project', 'cc4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/glossary',
                component: ComponentCreator('/physical-ai-course/docs/glossary', '465'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/guides/capstone-guide',
                component: ComponentCreator('/physical-ai-course/docs/guides/capstone-guide', '510'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/instructors/overview',
                component: ComponentCreator('/physical-ai-course/docs/instructors/overview', '8b5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/intro',
                component: ComponentCreator('/physical-ai-course/docs/intro', '405'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-1-ros-nervous-system/',
                component: ComponentCreator('/physical-ai-course/docs/module-1-ros-nervous-system/', 'd1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-1-ros-nervous-system/deliverables/deliverable-1',
                component: ComponentCreator('/physical-ai-course/docs/module-1-ros-nervous-system/deliverables/deliverable-1', '38b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-1-ros-nervous-system/labs/lab-1',
                component: ComponentCreator('/physical-ai-course/docs/module-1-ros-nervous-system/labs/lab-1', '3ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-1-ros-nervous-system/learning-outcomes',
                component: ComponentCreator('/physical-ai-course/docs/module-1-ros-nervous-system/learning-outcomes', '5d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-1-ros-nervous-system/lessons/lesson-1',
                component: ComponentCreator('/physical-ai-course/docs/module-1-ros-nervous-system/lessons/lesson-1', '041'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-2-digital-twin/',
                component: ComponentCreator('/physical-ai-course/docs/module-2-digital-twin/', '1bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-2-digital-twin/deliverables/deliverable-1',
                component: ComponentCreator('/physical-ai-course/docs/module-2-digital-twin/deliverables/deliverable-1', '3cc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-2-digital-twin/labs/lab-1',
                component: ComponentCreator('/physical-ai-course/docs/module-2-digital-twin/labs/lab-1', 'be5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-2-digital-twin/learning-outcomes',
                component: ComponentCreator('/physical-ai-course/docs/module-2-digital-twin/learning-outcomes', 'e14'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-2-digital-twin/lessons/lesson-1',
                component: ComponentCreator('/physical-ai-course/docs/module-2-digital-twin/lessons/lesson-1', 'd42'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-3-nvidia-isaac-brain/',
                component: ComponentCreator('/physical-ai-course/docs/module-3-nvidia-isaac-brain/', '088'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-3-nvidia-isaac-brain/deliverables/deliverable-1',
                component: ComponentCreator('/physical-ai-course/docs/module-3-nvidia-isaac-brain/deliverables/deliverable-1', '7ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-3-nvidia-isaac-brain/labs/lab-1',
                component: ComponentCreator('/physical-ai-course/docs/module-3-nvidia-isaac-brain/labs/lab-1', '643'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-3-nvidia-isaac-brain/learning-outcomes',
                component: ComponentCreator('/physical-ai-course/docs/module-3-nvidia-isaac-brain/learning-outcomes', '05a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-3-nvidia-isaac-brain/lessons/lesson-1',
                component: ComponentCreator('/physical-ai-course/docs/module-3-nvidia-isaac-brain/lessons/lesson-1', 'c2a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-4-vision-language-action/',
                component: ComponentCreator('/physical-ai-course/docs/module-4-vision-language-action/', 'b3a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-4-vision-language-action/deliverables/deliverable-1',
                component: ComponentCreator('/physical-ai-course/docs/module-4-vision-language-action/deliverables/deliverable-1', 'bce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-4-vision-language-action/labs/lab-1',
                component: ComponentCreator('/physical-ai-course/docs/module-4-vision-language-action/labs/lab-1', 'c17'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-4-vision-language-action/learning-outcomes',
                component: ComponentCreator('/physical-ai-course/docs/module-4-vision-language-action/learning-outcomes', '6b7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/module-4-vision-language-action/lessons/lesson-1',
                component: ComponentCreator('/physical-ai-course/docs/module-4-vision-language-action/lessons/lesson-1', 'cea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/reference/ros-isaac-components',
                component: ComponentCreator('/physical-ai-course/docs/reference/ros-isaac-components', '55b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/timeline',
                component: ComponentCreator('/physical-ai-course/docs/timeline', 'bb6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/congratulations', '05a'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/create-a-blog-post', '277'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/create-a-document', '661'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/create-a-page', '2c5'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/deploy-your-site', '2f7'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-basics/markdown-features', 'ec6'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-extras/manage-docs-versions', '166'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/physical-ai-course/docs/tutorial-extras/translate-your-site', 'b0f'),
                exact: true
              },
              {
                path: '/physical-ai-course/docs/tutorials/first-ros2-package',
                component: ComponentCreator('/physical-ai-course/docs/tutorials/first-ros2-package', 'b29'),
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
    path: '/physical-ai-course/',
    component: ComponentCreator('/physical-ai-course/', '5d7'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
