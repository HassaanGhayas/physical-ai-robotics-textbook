# Quickstart Guide: UI/UX Design System

**Date**: 2025-12-18  
**For**: Developers implementing the design system  
**Prerequisites**: Node.js 20+, npm, basic React/TypeScript knowledge

## 🚀 Quick Start (5 minutes)

### 1. Install Dependencies

```bash
# Tailwind CSS + PostCSS
npm install -D tailwindcss@^3.4 postcss autoprefixer

# shadcn/ui utilities
npm install -D @types/node
npm install class-variance-authority clsx tailwind-merge lucide-react

# Animation libraries
npm install framer-motion@^11 lottie-react

# Fonts
npm install @fontsource-variable/inter
```

### 2. Create Docusaurus Plugins

Create `plugins/tailwind-config.cjs`:
```javascript
module.exports = function () {
  return {
    name: 'tailwind-plugin',
    configurePostCss(postcssOptions) {
      postcssOptions.plugins.push(
        require('tailwindcss'),
        require('autoprefixer'),
      );
      return postcssOptions;
    },
  };
};
```

Create `plugins/alias-plugin.cjs`:
```javascript
const path = require('path');

module.exports = function () {
  return {
    name: 'alias-plugin',
    configureWebpack() {
      return {
        resolve: {
          alias: {
            '@': path.resolve(__dirname, '../src'),
          },
        },
      };
    },
  };
};
```

### 3. Update Configuration

Update `docusaurus.config.ts`:
```typescript
plugins: [
  './plugins/tailwind-config.cjs',
  './plugins/alias-plugin.cjs',
],
```

Update `src/css/custom.css`:
```css
@import '@fontsource-variable/inter';
@tailwind base;
@tailwind components;
@tailwind utilities;
```

### 4. Initialize shadcn/ui

```bash
npx shadcn@latest init
```

Select options:
- TypeScript: Yes
- Path: `@/components`
- CSS variables: Yes

### 5. Start Development

```bash
npm start
```

---

## 📚 Complete Setup Guide

[See full documentation in plan.md for detailed configuration]

---

## 🎨 Usage Examples

### Using Design Tokens
```tsx
import { spacing, colors } from '@/design-system/tokens';

<div style={{ padding: spacing[6], color: colors.text.primary }}>
  Content
</div>
```

### Using Animations
```tsx
import { FadeIn } from '@/design-system/animations';

<FadeIn duration="slow">
  <h1>Animated Heading</h1>
</FadeIn>
```

### Using Bento Grid
```tsx
import { BentoGrid } from '@/design-system/components';

<BentoGrid items={gridItems} />
```

---

## 📖 Next Steps

1. Review contracts/component-api.ts for component interfaces
2. Review data-model.md for design tokens
3. Follow implementation tasks in tasks.md (when available)

