---
id: 9
title: Card Mobile Responsiveness Fix
stage: green
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: ui-ux-design
branch: 001-ui-ux-design
user: hasss
command: /sp.implement
labels: ["mobile", "responsiveness", "css", "card", "testing"]
links:
  spec: D:\my-web\specs\001-ui-ux-design\spec.md
  ticket: null
  adr: null
  pr: null
files_modified:
  - D:\my-web\src\components\HardwareSpecs\HardwareSpecs.module.css
  - D:\my-web\tests\card-responsive.spec.ts
  - D:\my-web\specs\001-ui-ux-design\tasks.md
tests_added:
  - tests\card-responsive.spec.ts (3 tests for iPhone SE 375px, 320px, and text wrapping)
---

## Prompt

```text
the card class="card_bm8r" breaks on small devices
```

## Response

Fixed mobile responsiveness issues for HardwareSpecs card component. The CSS module class (card_bm8r is the generated hash) was breaking on small devices due to missing mobile constraints and text wrapping rules.

### Changes Made

**File: src/components/HardwareSpecs/HardwareSpecs.module.css**

1. Added base card constraints:
   - `width: 100%; max-width: 100%; min-width: 0` - Ensures card never exceeds viewport
   - `overflow-wrap: break-word; word-wrap: break-word; hyphens: auto` - Proper text wrapping

2. Enhanced mobile media queries:
   - **@media (max-width: 768px)**: Reduced padding to 1.25rem
   - **@media (max-width: 480px)**:
     - Reduced padding to 1rem
     - Reduced border-radius to 6px
     - Stacked header vertically (flex-direction: column)
     - Reduced font sizes (h3: 1.1rem, h4: 0.85rem, p: 0.9rem)
     - Reduced container padding to 0.5rem

### Testing

Created comprehensive test suite (`tests/card-responsive.spec.ts`) covering:
- ✅ iPhone SE (375px): Zero horizontal scrolling
- ✅ Very small screen (320px): Zero horizontal scrolling
- ✅ Text wrapping: Verified overflow-wrap and word-wrap applied correctly

All 3 Playwright tests passed successfully.

### Important Note

**NO `overflow: hidden` was added**. The fix ensures content genuinely fits through:
- Flexible width constraints
- Proper text wrapping
- Responsive padding/font reductions
- Vertical stacking on mobile

This prevents masking layout issues while ensuring cards display correctly on all device sizes.

### Files Modified

- src/components/HardwareSpecs/HardwareSpecs.module.css
- tests/card-responsive.spec.ts (new)
- specs/001-ui-ux-design/tasks.md (marked T041-T042 complete)

### Tasks Completed

- T041: Card component with variants ✓
- T042: Card styles with mobile responsiveness ✓
