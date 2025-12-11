import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/blog',
    component: ComponentCreator('/blog', 'de3'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '89a'),
    exact: true
  },
  {
    path: '/blog/iqra-author-introduction',
    component: ComponentCreator('/blog/iqra-author-introduction', '358'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '9ad'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', 'e9f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/ai',
    component: ComponentCreator('/blog/tags/ai', '3de'),
    exact: true
  },
  {
    path: '/blog/tags/author',
    component: ComponentCreator('/blog/tags/author', 'c52'),
    exact: true
  },
  {
    path: '/blog/tags/beginner',
    component: ComponentCreator('/blog/tags/beginner', '8af'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '30d'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '0f9'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '00d'),
    exact: true
  },
  {
    path: '/blog/tags/introduction',
    component: ComponentCreator('/blog/tags/introduction', 'e75'),
    exact: true
  },
  {
    path: '/blog/tags/journey',
    component: ComponentCreator('/blog/tags/journey', 'a3f'),
    exact: true
  },
  {
    path: '/blog/tags/programming',
    component: ComponentCreator('/blog/tags/programming', '716'),
    exact: true
  },
  {
    path: '/blog/tags/robotics',
    component: ComponentCreator('/blog/tags/robotics', 'aa4'),
    exact: true
  },
  {
    path: '/blog/tags/web-development',
    component: ComponentCreator('/blog/tags/web-development', 'ae3'),
    exact: true
  },
  {
    path: '/blog/welcome-anusha',
    component: ComponentCreator('/blog/welcome-anusha', '95c'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '66f'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'a69'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'c44'),
            routes: [
              {
                path: '/docs/capstone-project',
                component: ComponentCreator('/docs/capstone-project', '2bf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/glossary',
                component: ComponentCreator('/docs/glossary', 'a11'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/guides/capstone-guide',
                component: ComponentCreator('/docs/guides/capstone-guide', '0d1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/instructors/overview',
                component: ComponentCreator('/docs/instructors/overview', 'a03'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros-nervous-system/',
                component: ComponentCreator('/docs/module-1-ros-nervous-system/', 'f4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros-nervous-system/deliverables/deliverable-1',
                component: ComponentCreator('/docs/module-1-ros-nervous-system/deliverables/deliverable-1', 'db2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros-nervous-system/labs/lab-1',
                component: ComponentCreator('/docs/module-1-ros-nervous-system/labs/lab-1', '182'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros-nervous-system/learning-outcomes',
                component: ComponentCreator('/docs/module-1-ros-nervous-system/learning-outcomes', '3e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros-nervous-system/lessons/lesson-1',
                component: ComponentCreator('/docs/module-1-ros-nervous-system/lessons/lesson-1', '76a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/',
                component: ComponentCreator('/docs/module-2-digital-twin/', 'b7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/deliverables/deliverable-1',
                component: ComponentCreator('/docs/module-2-digital-twin/deliverables/deliverable-1', '9e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/labs/lab-1',
                component: ComponentCreator('/docs/module-2-digital-twin/labs/lab-1', 'e6c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/learning-outcomes',
                component: ComponentCreator('/docs/module-2-digital-twin/learning-outcomes', 'b92'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/lessons/lesson-1',
                component: ComponentCreator('/docs/module-2-digital-twin/lessons/lesson-1', 'c24'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-nvidia-isaac-brain/',
                component: ComponentCreator('/docs/module-3-nvidia-isaac-brain/', '8fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-nvidia-isaac-brain/deliverables/deliverable-1',
                component: ComponentCreator('/docs/module-3-nvidia-isaac-brain/deliverables/deliverable-1', 'f02'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-nvidia-isaac-brain/labs/lab-1',
                component: ComponentCreator('/docs/module-3-nvidia-isaac-brain/labs/lab-1', '07e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-nvidia-isaac-brain/learning-outcomes',
                component: ComponentCreator('/docs/module-3-nvidia-isaac-brain/learning-outcomes', '7cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-nvidia-isaac-brain/lessons/lesson-1',
                component: ComponentCreator('/docs/module-3-nvidia-isaac-brain/lessons/lesson-1', 'a9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vision-language-action/',
                component: ComponentCreator('/docs/module-4-vision-language-action/', '69a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vision-language-action/deliverables/deliverable-1',
                component: ComponentCreator('/docs/module-4-vision-language-action/deliverables/deliverable-1', '889'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vision-language-action/labs/lab-1',
                component: ComponentCreator('/docs/module-4-vision-language-action/labs/lab-1', '7e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vision-language-action/learning-outcomes',
                component: ComponentCreator('/docs/module-4-vision-language-action/learning-outcomes', '404'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vision-language-action/lessons/lesson-1',
                component: ComponentCreator('/docs/module-4-vision-language-action/lessons/lesson-1', '5f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/reference/ros-isaac-components',
                component: ComponentCreator('/docs/reference/ros-isaac-components', '49f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/timeline',
                component: ComponentCreator('/docs/timeline', 'd3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/docs/tutorial-basics/congratulations', '70e'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post', '315'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document', 'f86'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page', '9f6'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site', 'b91'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features', '272'),
                exact: true
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions', 'a34'),
                exact: true
              },
              {
                path: '/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site', '739'),
                exact: true
              },
              {
                path: '/docs/tutorials/first-ros2-package',
                component: ComponentCreator('/docs/tutorials/first-ros2-package', '6bc'),
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
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
