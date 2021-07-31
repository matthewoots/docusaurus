const math = require('remark-math');
const katex = require('rehype-katex');
const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

module.exports = {
  title: 'Personal Research & Development',
  tagline: 'Documentation Website',
  url: 'https://matthewoots.github.io',
  baseUrl: '/docusaurus/',
  onBrokenLinks: 'ignore', // throw
  favicon: 'img/logo.png',
  organizationName: 'matthewoots', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.
  stylesheets: [    
    {      
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.11/dist/katex.min.css',      
      integrity:        
        'sha384-Um5gpz1odJg5Z4HAmzPtgZKdTBHZdw8S29IecapCSB31ligYPhHQZMIlWLYQGVoc',      
      crossorigin: 'anonymous',    
    },  
  ],
  
  themeConfig: {
    prism: {
      additionalLanguages: ['cpp'],
      theme: require('prism-react-renderer/themes/github'),
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: true,
    },
    navbar: {
      title: 'Main',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          to: 'docs/px4-autopilot/introduction/intro',
          activeBasePath: 'px4-autopilot',
          label: 'PX4-Autopilot',
          position: 'left',
        },
        {
          to: 'docs/motion-planning/introduction/intro',
          activeBasePath: 'motion-planning',
          label: 'Motion-Planning',
          position: 'left',
        },
        {
          to: 'docs/gazebo/introduction/intro',
          activeBasePath: 'gazebo',
          label: 'Gazebo',
          position: 'left',
        },
        {
          to: 'docs/unity/introduction/intro',
          activeBasePath: 'unity',
          label: 'Unity3D',
          position: 'left',
        },
        {
          to: 'docs/Docusaurus/intro',
          activeBasePath: 'docs',
          label: 'Docs',
          position: 'left',
        },
        {to: 'blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/matthewoots',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    algolia: {
      apiKey: 'e44c59d36394a26b1616e631a1c9d197',
      indexName: 'edy',
      appId: 'HXJSB0I3EO',

      // Optional: see doc section bellow
      // contextualSearch: true,

      //... other Algolia params
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Introduction',
              to: 'docs/unity/introduction/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: 'blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/facebook/docusaurus',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Built with Docusaurus.`,
    },
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          // It is recommended to set document id as docs home page (`docs/` path).
          // homePageId: '/Docusaurus/intro',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          editUrl:
            'https://github.com/facebook/docusaurus/edit/master/website/',
          showLastUpdateTime: true,
          remarkPlugins: [math],          
          rehypePlugins: [katex],
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          editUrl:
            'https://github.com/facebook/docusaurus/edit/master/website/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
