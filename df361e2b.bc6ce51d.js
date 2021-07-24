(window.webpackJsonp=window.webpackJsonp||[]).push([[32],{89:function(e,t,n){"use strict";n.r(t),n.d(t,"frontMatter",(function(){return r})),n.d(t,"metadata",(function(){return s})),n.d(t,"rightToc",(function(){return c})),n.d(t,"default",(function(){return l}));var a=n(2),i=n(6),o=(n(0),n(96)),r={id:"doc2",title:"Introduction",sidebar_label:"Introduction"},s={unversionedId:"doc2",id:"doc2",isDocsHomePage:!1,title:"Introduction",description:"Overview",source:"@site/docs/doc2.md",permalink:"/docusaurus/docs/doc2",editUrl:"https://github.com/facebook/docusaurus/edit/master/website/docs/doc2.md",sidebar_label:"Introduction"},c=[{value:"Overview",id:"overview",children:[{value:"Access the Folder in VS Code (Remote SSH)",id:"access-the-folder-in-vs-code-remote-ssh",children:[]},{value:"Access the Folder (as Files)",id:"access-the-folder-as-files",children:[]},{value:"File Functions",id:"file-functions",children:[]}]},{value:"Create a Page",id:"create-a-page",children:[]},{value:"Add a Page to the Sidebar",id:"add-a-page-to-the-sidebar",children:[]},{value:"Add Important Pages to Navigation Bar",id:"add-important-pages-to-navigation-bar",children:[]},{value:"Preview Your Changes",id:"preview-your-changes",children:[]}],d={rightToc:c};function l(e){var t=e.components,n=Object(i.a)(e,["components"]);return Object(o.b)("wrapper",Object(a.a)({},d,n,{components:t,mdxType:"MDXLayout"}),Object(o.b)("h2",{id:"overview"},"Overview"),Object(o.b)("p",null,"The pages are available as source code from our server, available as remote folder. It can be accessed by VS Code directly, or using mounted network drive option."),Object(o.b)("h3",{id:"access-the-folder-in-vs-code-remote-ssh"},"Access the Folder in VS Code (Remote SSH)"),Object(o.b)("div",{className:"admonition admonition-tip alert alert--success"},Object(o.b)("div",Object(a.a)({parentName:"div"},{className:"admonition-heading"}),Object(o.b)("h5",{parentName:"div"},Object(o.b)("span",Object(a.a)({parentName:"h5"},{className:"admonition-icon"}),Object(o.b)("svg",Object(a.a)({parentName:"span"},{xmlns:"http://www.w3.org/2000/svg",width:"12",height:"16",viewBox:"0 0 12 16"}),Object(o.b)("path",Object(a.a)({parentName:"svg"},{fillRule:"evenodd",d:"M6.5 0C3.48 0 1 2.19 1 5c0 .92.55 2.25 1 3 1.34 2.25 1.78 2.78 2 4v1h5v-1c.22-1.22.66-1.75 2-4 .45-.75 1-2.08 1-3 0-2.81-2.48-5-5.5-5zm3.64 7.48c-.25.44-.47.8-.67 1.11-.86 1.41-1.25 2.06-1.45 3.23-.02.05-.02.11-.02.17H5c0-.06 0-.13-.02-.17-.2-1.17-.59-1.83-1.45-3.23-.2-.31-.42-.67-.67-1.11C2.44 6.78 2 5.65 2 5c0-2.2 2.02-4 4.5-4 1.22 0 2.36.42 3.22 1.19C10.55 2.94 11 3.94 11 5c0 .66-.44 1.78-.86 2.48zM4 14h5c-.23 1.14-1.3 2-2.5 2s-2.27-.86-2.5-2z"})))),"tip")),Object(o.b)("div",Object(a.a)({parentName:"div"},{className:"admonition-content"}),Object(o.b)("p",{parentName:"div"},"This is the more convenient way of editting the documentation files, and at the same time obtain a command prompt."))),Object(o.b)("p",null,"In VS Code, install plugin ",Object(o.b)("inlineCode",{parentName:"p"},"Remote - SSH"),". Then at the bottom-left corner, press the green button ",Object(o.b)("inlineCode",{parentName:"p"},"open a remote window"),", follow the prompt at the top to proceed for the user at host (",Object(o.b)("inlineCode",{parentName:"p"},"tsluser@172.18.72.192"),"), password (given to you) key-ins."),Object(o.b)("blockquote",null,Object(o.b)("p",{parentName:"blockquote"},"The website folder is located at ",Object(o.b)("inlineCode",{parentName:"p"},"~/docusaurus_html/tl-tech-details"),".")),Object(o.b)("h3",{id:"access-the-folder-as-files"},"Access the Folder (as Files)"),Object(o.b)("p",null,"All website generation files is located at our server within NUS intranet (VPN is needed if accessing outside NUS)"),Object(o.b)("ul",null,Object(o.b)("li",{parentName:"ul"},Object(o.b)("inlineCode",{parentName:"li"},"smb://172.18.72.192/techdetails/")," for Linux, and"),Object(o.b)("li",{parentName:"ul"},Object(o.b)("inlineCode",{parentName:"li"},"\\\\172.18.72.192\\techdetails")," for Windows (",Object(o.b)("strong",{parentName:"li"},"Mount as network drive, instead of a network location"),")")),Object(o.b)("p",null,"| Folder displayed in Linux| Mounted Drive in Windows |"),Object(o.b)("div",{className:"admonition admonition-note alert alert--secondary"},Object(o.b)("div",Object(a.a)({parentName:"div"},{className:"admonition-heading"}),Object(o.b)("h5",{parentName:"div"},Object(o.b)("span",Object(a.a)({parentName:"h5"},{className:"admonition-icon"}),Object(o.b)("svg",Object(a.a)({parentName:"span"},{xmlns:"http://www.w3.org/2000/svg",width:"14",height:"16",viewBox:"0 0 14 16"}),Object(o.b)("path",Object(a.a)({parentName:"svg"},{fillRule:"evenodd",d:"M6.3 5.69a.942.942 0 0 1-.28-.7c0-.28.09-.52.28-.7.19-.18.42-.28.7-.28.28 0 .52.09.7.28.18.19.28.42.28.7 0 .28-.09.52-.28.7a1 1 0 0 1-.7.3c-.28 0-.52-.11-.7-.3zM8 7.99c-.02-.25-.11-.48-.31-.69-.2-.19-.42-.3-.69-.31H6c-.27.02-.48.13-.69.31-.2.2-.3.44-.31.69h1v3c.02.27.11.5.31.69.2.2.42.31.69.31h1c.27 0 .48-.11.69-.31.2-.19.3-.42.31-.69H8V7.98v.01zM7 2.3c-3.14 0-5.7 2.54-5.7 5.68 0 3.14 2.56 5.7 5.7 5.7s5.7-2.55 5.7-5.7c0-3.15-2.56-5.69-5.7-5.69v.01zM7 .98c3.86 0 7 3.14 7 7s-3.14 7-7 7-7-3.12-7-7 3.14-7 7-7z"})))),"Regarding where to put in the address")),Object(o.b)("div",Object(a.a)({parentName:"div"},{className:"admonition-content"}),Object(o.b)("p",{parentName:"div"},"On Linux machines, go to the file explorer, in the location bar, key in the address and press enter."),Object(o.b)("p",{parentName:"div"},"On Windows machines, go to file explorer --\x3e This PC (Computer) --\x3e Map network drive, then key in the credentials (switch domain to WORKGROUP)"))),Object(o.b)("h3",{id:"file-functions"},"File Functions"),Object(o.b)("h4",{id:"md-file"},Object(o.b)("inlineCode",{parentName:"h4"},".md")," file"),Object(o.b)("p",null,"Each page is one ",Object(o.b)("inlineCode",{parentName:"p"},".md")," file located in the ",Object(o.b)("inlineCode",{parentName:"p"},"docs")," subfolder, arranged according to the navigation bar (e.g. ",Object(o.b)("inlineCode",{parentName:"p"},"./docs/hardware/")," and ",Object(o.b)("inlineCode",{parentName:"p"},"./docs/systems"),")."),Object(o.b)("h4",{id:"sidebarsjs-file"},Object(o.b)("inlineCode",{parentName:"h4"},"sidebars.js")," file"),Object(o.b)("p",null,"To make the page appear on the sidebar, file ",Object(o.b)("inlineCode",{parentName:"p"},"./docs/sidebars.js")," requires modification"),Object(o.b)("h4",{id:"docusaurusconfigjs-file"},Object(o.b)("inlineCode",{parentName:"h4"},"docusaurus.config.js")," file"),Object(o.b)("p",null,"In rarer cases, if a new tab or sub-entries need to be created at the navigation bar (on the top of the website), ",Object(o.b)("inlineCode",{parentName:"p"},"docusaurus.config.js")," requires modification"),Object(o.b)("h2",{id:"create-a-page"},"Create a Page"),Object(o.b)("p",null,"Creating a page is as simple as by creating a file like ",Object(o.b)("inlineCode",{parentName:"p"},"mypage.md"),". For example, if we create the file under the path ",Object(o.b)("inlineCode",{parentName:"p"},"./docs/examples/mypage.md"),", then the page should be immediately accessible through the live preview at port 8888:\n",Object(o.b)("inlineCode",{parentName:"p"},"http://172.18.72.192:8888/tech-details/docs/examples/mypage")),Object(o.b)("p",null,"Within the empty ",Object(o.b)("inlineCode",{parentName:"p"},"mypage.md"),", add the front matter to configure the title"),Object(o.b)("pre",null,Object(o.b)("code",Object(a.a)({parentName:"pre"},{className:"language-markdown"}),"---\nhide_title: true\nsidebar_label: My Page\n---\n\n# My Title\n\nThe rest of your content...\n")),Object(o.b)("p",null,"The end result should look like this:"),Object(o.b)("p",null,"Notice that there is no sidebar available for this page, which means the page is currently not browsable unless the exact URL is keyed in. Therefore we will add the page to sidebar next."),Object(o.b)("h2",{id:"add-a-page-to-the-sidebar"},"Add a Page to the Sidebar"),Object(o.b)("p",null,"The file ",Object(o.b)("inlineCode",{parentName:"p"},"./docs/sidebars.js")," is organised in blocks like this:"),Object(o.b)("pre",null,Object(o.b)("code",Object(a.a)({parentName:"pre"},{className:"language-js"}),"    // systems\n    systemsSidebar: {\n        'Past Platforms : ddrone v2': [\n            'systems/ddrone_v2/ddrone',\n        ],\n        'Simulations': [\n            'systems/simulation/unity-SITL',\n            'systems/simulation/unity-HITL',\n        ],\n        'Vicon': [\n            'systems/vicon',\n        ],\n    },\n")),Object(o.b)("p",null,"Each block has a name like ",Object(o.b)("inlineCode",{parentName:"p"},"systemsSidebar")," which is arbitrary (does not affect the website display), but it logically links all relevant documents under one sidebar group."),Object(o.b)("p",null,"Within each block, there can be nested levels of sidebar entries, and the format should be self-explanatory. But be reminded of the ",Object(o.b)("inlineCode",{parentName:"p"},",")," at various places for the correct syntax. The ",Object(o.b)("inlineCode",{parentName:"p"},".md")," postfix should be omitted when mentioning each file."),Object(o.b)("p",null,"After adding content in ",Object(o.b)("inlineCode",{parentName:"p"},"sidebars.js"),", the page should now be rendered like this:"),Object(o.b)("h2",{id:"add-important-pages-to-navigation-bar"},"Add Important Pages to Navigation Bar"),Object(o.b)("p",null,"The navigation bar is configured within the main ",Object(o.b)("inlineCode",{parentName:"p"},"docusaurus.config.js")," file. It is under the object ",Object(o.b)("inlineCode",{parentName:"p"},"themeConfig.navbar.items"),"."),Object(o.b)("p",null,"Each item can be nested, an example would be:"),Object(o.b)("pre",null,Object(o.b)("code",Object(a.a)({parentName:"pre"},{className:"language-js"}),"    // Systems\n    {\n        to: 'docs/systems/vicon',\n        activeBasePath: 'docs/systems',\n        label: 'Systems',\n        position: 'left',\n        items: [\n        \n        {\n            // activeBasePath:'docs/systems/',\n            label: 'Simulations',\n            to: 'docs/systems/simulation/unity-SITL'\n        },\n        {\n            // activeBasePath:'docs/systems/',\n            label: 'Vicon',\n            to: 'docs/systems/vicon'\n        },\n        {\n            // activeBasePath:'docs/systems/ddrone_v2',\n            label: 'Past Platforms : ddrone v2',\n            to: 'docs/systems/ddrone_v2/ddrone'\n        },\n        \n        ]\n    },\n")),Object(o.b)("p",null,"Again the syntax should be self-explanatory."),Object(o.b)("h2",{id:"preview-your-changes"},"Preview Your Changes"),Object(o.b)("p",null,"Preview your live changes at ",Object(o.b)("inlineCode",{parentName:"p"},"http://172.18.72.192:8888/tech-details/")))}l.isMDXComponent=!0},96:function(e,t,n){"use strict";n.d(t,"a",(function(){return b})),n.d(t,"b",(function(){return u}));var a=n(0),i=n.n(a);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function r(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function s(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?r(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):r(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function c(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var d=i.a.createContext({}),l=function(e){var t=i.a.useContext(d),n=t;return e&&(n="function"==typeof e?e(t):s(s({},t),e)),n},b=function(e){var t=l(e.components);return i.a.createElement(d.Provider,{value:t},e.children)},p={inlineCode:"code",wrapper:function(e){var t=e.children;return i.a.createElement(i.a.Fragment,{},t)}},m=i.a.forwardRef((function(e,t){var n=e.components,a=e.mdxType,o=e.originalType,r=e.parentName,d=c(e,["components","mdxType","originalType","parentName"]),b=l(n),m=a,u=b["".concat(r,".").concat(m)]||b[m]||p[m]||o;return n?i.a.createElement(u,s(s({ref:t},d),{},{components:n})):i.a.createElement(u,s({ref:t},d))}));function u(e,t){var n=arguments,a=t&&t.mdxType;if("string"==typeof e||a){var o=n.length,r=new Array(o);r[0]=m;var s={};for(var c in t)hasOwnProperty.call(t,c)&&(s[c]=t[c]);s.originalType=e,s.mdxType="string"==typeof e?e:a,r[1]=s;for(var d=2;d<o;d++)r[d]=n[d];return i.a.createElement.apply(null,r)}return i.a.createElement.apply(null,n)}m.displayName="MDXCreateElement"}}]);