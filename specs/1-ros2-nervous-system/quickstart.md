# Quickstart Guide: The Robotic Nervous System (ROS 2)

**Feature**: 1-ros2-nervous-system
**Date**: 2025-12-17
**Status**: Completed

## Overview

This quickstart guide provides everything needed to set up, run, and contribute to the ROS 2 educational module documentation. The documentation is built with Docusaurus, a modern static website generator optimized for documentation.

## Prerequisites

Before starting, ensure you have the following installed:

- **Node.js**: Version 18.x or higher
- **npm** or **yarn**: Package manager (npm comes with Node.js)
- **Git**: Version control system
- **A code editor**: VS Code, Vim, or your preferred editor

## Setting Up the Development Environment

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
npm install
# OR if using yarn
yarn install
```

### 3. Start the Development Server

```bash
npm start
# OR if using yarn
yarn start
```

This command starts a local development server and opens the documentation in your default browser at `http://localhost:3000`. Most changes are reflected live without restarting the server.

## Project Structure

The documentation follows Docusaurus conventions:

```
.
├── docs/                    # Documentation files (Markdown)
│   ├── intro.md            # Introduction page
│   ├── module-1-ros2-nervous-system/
│   │   ├── index.md        # Module overview
│   │   ├── chapter-1-intro-to-ros2.md
│   │   ├── chapter-2-communication-model.md
│   │   └── chapter-3-urdf-structure.md
│   └── ...
├── static/                 # Static assets (images, files)
├── src/                    # Custom React components
├── docusaurus.config.js    # Site configuration
├── sidebars.js             # Navigation structure
├── package.json            # Dependencies and scripts
└── README.md              # Project overview
```

## Creating New Content

### Adding a New Document

1. Create a new `.md` file in the appropriate directory under `docs/`
2. Add frontmatter at the top of the file:

```markdown
---
title: Your Document Title
description: Brief description of the document
sidebar_label: Label for sidebar
sidebar_position: Position in sidebar (1, 2, 3...)
tags: [tag1, tag2, tag3]
---

# Your Document Title

Your content here...
```

### Adding to Navigation

To add your document to the sidebar navigation, update `sidebars.js`:

```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2-nervous-system/index',
        'module-1-ros2-nervous-system/chapter-1-intro-to-ros2',
        'module-1-ros2-nervous-system/chapter-2-communication-model',
        'module-1-ros2-nervous-system/chapter-3-urdf-structure',
        'path-to-your-new-document'  // Add your document here
      ],
    },
    // ... other sections
  ],
};
```

## Writing Content

### Markdown Syntax

Docusaurus supports standard Markdown plus additional features:

```markdown
## Headers

Regular paragraph text.

**Bold text** and *italic text*.

- List item 1
- List item 2

1. Numbered item 1
2. Numbered item 2

[Link text](path/to/page)

![Image alt text](/img/image.png)
```

### Code Blocks

```markdown
Use triple backticks for code blocks:

\```javascript
function example() {
  console.log('Hello, world!');
}
\```

You can specify the language for syntax highlighting.
```

### Admonitions

Docusaurus supports special blocks for notes, tips, warnings, etc.:

```markdown
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tip>

This is a tip for the reader.

</Tip>

<Note>

This is an important note.

</Note>

<Warning>

This is a warning about potential issues.

</Warning>
```

### Adding Images

Place images in the `static/img/` directory and reference them with `/img/` prefix:

```markdown
![Alt text](/img/your-image.png)
```

## Building for Production

To build the static site for deployment:

```bash
npm run build
# OR if using yarn
yarn build
```

This command generates a `build/` directory with the complete static site that can be deployed to any static hosting service.

## Running Tests

To check for broken links and other issues:

```bash
npm run serve
# OR if using yarn
yarn serve
```

This serves the built site locally for testing.

## Contributing to the ROS 2 Module

### Chapter Structure

The ROS 2 module follows this structure:

1. **Chapter 1: Introduction to ROS 2 for Physical AI**
   - Focus: Core concepts and DDS
   - Target: Understanding ROS 2 fundamentals

2. **Chapter 2: ROS 2 Communication Model**
   - Focus: Nodes, topics, services with rclpy
   - Target: Practical communication implementation

3. **Chapter 3: Robot Structure with URDF**
   - Focus: Robot description and simulation
   - Target: Creating humanoid robot models

### Content Guidelines

- **Audience**: AI students and developers entering humanoid robotics
- **Tone**: Educational, clear, beginner-friendly
- **Examples**: Use humanoid robotics as consistent use case
- **Depth**: Explain concepts before providing practical examples
- **Validation**: Include exercises and quizzes to verify understanding

### Style Guide

- Use active voice when possible
- Keep paragraphs short (2-3 sentences)
- Use headings to organize content hierarchically
- Include code examples with explanations
- Add diagrams for complex concepts
- Link to official ROS 2 documentation for deeper learning

## Deployment

The site can be deployed to various platforms:

### GitHub Pages (Recommended)
- Push changes to the `main` branch
- GitHub Actions will automatically build and deploy
- Configure in `docusaurus.config.js` under `deploymentBranch`

### Other Platforms
- **Netlify**: Upload the `build/` directory
- **Vercel**: Connect to your Git repository
- **AWS S3**: Upload the `build/` directory with static hosting

## Troubleshooting

### Common Issues

**Q: The development server doesn't start**
A: Ensure Node.js and npm are properly installed. Run `node --version` and `npm --version` to verify.

**Q: Changes don't appear in the browser**
A: Check for syntax errors in your Markdown files. The development server will show error messages in the console.

**Q: Images don't display**
A: Ensure images are in the `static/` directory and referenced with the `/` prefix.

**Q: Sidebar navigation doesn't update**
A: Verify that your document path is correctly added to `sidebars.js`.

## Getting Help

- **Documentation**: Refer to the [Docusaurus documentation](https://docusaurus.io/docs)
- **Issues**: Report problems in the GitHub repository
- **Questions**: Ask in the project's discussion forum