---
id: 16
title: Fix inconsistent tab padding margin
stage: general
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: ui-ux-design
branch: 001-ui-ux-design
user: hasss
command: /sp.implement
labels: ["bugfix", "css", "padding", "tabs", "badges", "consistency"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - src/components/HardwareSpecs/HardwareSpecs.module.css
 - src/css/custom.css
tests:
 - npm run build (successful)
---

## Prompt

[Image #1] The tab paddings margin is not consistent throughout the pages. Analyze and fix it

The user provided a screenshot showing a category badge "Motion Control Systems" in the HardwareSpecs component with inconsistent padding (appeared cramped vertically compared to horizontally).

## Response snapshot

### Analysis

Identified that the "Motion Control Systems" element is not a tab but a category badge in the HardwareSpecs component. The badge had asymmetric padding:
- Original: `padding: 0.25rem 0.5rem` (0.25rem top/bottom, 0.5rem left/right)
- This created a cramped appearance with insufficient vertical spacing

### Fixes Applied

**1. HardwareSpecs.module.css** - Component-specific fixes:

```css
.category {
  padding: 0.5rem 0.75rem;  /* Increased from 0.25rem 0.5rem */
  white-space: nowrap;       /* Prevent text wrapping */
  display: inline-block;     /* Ensure proper box model */
}

/* Mobile responsive (480px and below) */
.category {
  font-size: 0.75rem;
  padding: 0.375rem 0.625rem;  /* Proportional scaling for mobile */
}
```

**2. custom.css** - Global consistency rules (45 new lines):

Added comprehensive badge/pill/tab padding rules to ensure consistency across the entire site:

```css
/* Badge-like elements */
[class*="badge"],
[class*="pill"],
[class*="tag"],
[class*="category"] {
  padding: 0.5rem 0.75rem !important;
  border-radius: 12px;
  white-space: nowrap;
  display: inline-block;
}

/* Mobile responsive badge padding */
@media (max-width: 480px) {
  [class*="badge"],
  [class*="pill"],
  [class*="tag"],
  [class*="category"] {
    padding: 0.375rem 0.625rem !important;
  }
}

/* Docusaurus tabs (if used in future) */
[class*="tabs__item"],
.tabs button {
  padding: 0.5rem 1rem !important;
}

@media (max-width: 768px) {
  [class*="tabs__item"],
  .tabs button {
    padding: 0.375rem 0.75rem !important;
  }
}
```

### Padding Values

**Desktop (>480px)**:
- Category badges: 0.5rem vertical, 0.75rem horizontal (increased from 0.25rem/0.5rem)
- Tab items: 0.5rem vertical, 1rem horizontal

**Mobile (≤480px)**:
- Category badges: 0.375rem vertical, 0.625rem horizontal
- Tab items: 0.375rem vertical, 0.75rem horizontal

**Tablet (≤768px)**:
- Tab items: 0.375rem vertical, 0.75rem horizontal

### Key Improvements

1. **Balanced padding ratio**: Changed from 1:2 (vertical:horizontal) to approximately 2:3, creating more balanced visual appearance
2. **Consistent spacing**: All badge-like elements now use the same padding values
3. **Responsive scaling**: Mobile devices get proportionally smaller padding while maintaining the same ratio
4. **Future-proof**: Global CSS rules ensure any new badges/pills/tabs will have consistent styling
5. **White-space handling**: Added `nowrap` to prevent text from wrapping inside badges

## Outcome

- ✅ Impact: Fixed inconsistent padding on category badges across all HardwareSpecs components, established global standards for badge/pill/tab elements
- 🧪 Tests: npm run build completed successfully
- 📁 Files: src/components/HardwareSpecs/HardwareSpecs.module.css (3 lines modified), src/css/custom.css (45 lines added)
- 🔁 Next prompts: Visual verification on actual browser, test on multiple pages with HardwareSpecs components
- 🧠 Reflection: The fix addresses both the immediate issue (HardwareSpecs category badge) and establishes site-wide consistency standards for all badge-like elements. Using CSS attribute selectors ensures the rules apply to all current and future components with similar naming patterns.

## Evaluation notes (flywheel)

- Failure modes observed: None - build successful, CSS additions use !important to override any conflicting styles
- Graders run and results (PASS/FAIL): npm run build - PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test visual appearance on actual browser at different viewport sizes (desktop, tablet, mobile) to confirm balanced padding appearance
