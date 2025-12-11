Fix the following errors:
[WARNING] The `siteConfig.onBrokenMarkdownLinks` config option is deprecated and will be removed in Docusaurus v4.
Please migrate and move this option to `siteConfig.markdown.hooks.onBrokenMarkdownLinks` instead.
[ERROR] Loading of version failed for version current

[ERROR] Error: Invalid sidebar file at "sidebars.ts".
These sidebar document ids do not exist:
- book/assessments/capstone-assessment
- book/assessments/isaac-pipeline
- book/intro

Available document ids are:
- book/ai-robot-brain/index
- book/ai-robot-brain/isaac-ros
- book/ai-robot-brain/isaac-sim
- book/ai-robot-brain/nav2-path-planning
- book/assessments/comprehensive-capstone
- book/assessments/gazebo-implementation
- book/assessments/index
- book/assessments/ros2-package-project
- book/digital-twin/gazebo-simulation
- book/digital-twin/index
- book/digital-twin/sensor-simulation
- book/digital-twin/unity-rendering
- book/hardware-requirements/cloud-alternatives
- book/hardware-requirements/digital-twin-workstation
- book/hardware-requirements/edge-kit
- book/hardware-requirements/index
- book/hardware-requirements/robot-lab-options
- book/introduction/index
- book/robotic-nervous-system/bridging-python-agents
- book/robotic-nervous-system/index
- book/robotic-nervous-system/ros2-nodes-topics-services
- book/robotic-nervous-system/urdf-for-humanoids
- book/vision-language-action/capstone-project
- book/vision-language-action/cognitive-planning
- book/vision-language-action/index
- book/vision-language-action/voice-to-action
- intro
- tutorial-basics/congratulations
- tutorial-basics/create-a-blog-post
- tutorial-basics/create-a-document
- tutorial-basics/create-a-page
- tutorial-basics/deploy-your-site
- tutorial-basics/markdown-features
- tutorial-extras/manage-docs-versions
- tutorial-extras/translate-your-site

    at Object.checkSidebarsDocIds (D:\my-web\node_modules\@docusaurus\plugin-content-docs\lib\sidebars\utils.js:245:19)
    at doLoadVersion (D:\my-web\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:102:19)
    at async loadVersion (D:\my-web\node_modules\@docusaurus\plugin-content-docs\lib\versions\loadVersion.js:119:16)
    at async Promise.all (index 0)
    at async Object.loadContent (D:\my-web\node_modules\@docusaurus\plugin-content-docs\lib\index.js:152:33)
    at async D:\my-web\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:40:23
    at async Promise.all (index 1)
    at async D:\my-web\node_modules\@docusaurus\core\lib\server\plugins\plugins.js:146:25
    at async loadSite (D:\my-web\node_modules\@docusaurus\core\lib\server\site.js:155:45)
    at async createReloadableSite (D:\my-web\node_modules\@docusaurus\core\lib\commands\start\utils.js:62:16)