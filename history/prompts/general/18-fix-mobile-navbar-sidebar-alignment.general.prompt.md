---
id: 18
title: Fix mobile navbar sidebar alignment
stage: general
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: ui-ux-design
branch: 001-ui-ux-design
user: hasss
command: /sp.implement
labels: ["bugfix", "css", "mobile", "sidebar", "alignment", "padding"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - src/css/custom.css
tests:
 - npm run build (successful)
---

## Prompt

class="theme-layout-navbar-sidebar-panel navbar-sidebar__item menu" align this properly

The links are too much sticked to the left

## Response snapshot

Fixed mobile navbar sidebar panel alignment by adding specific styles for the `theme-layout-navbar-sidebar-panel` and `navbar-sidebar__item menu` classes with proper padding and vertical centering.

### Problem Analysis

The mobile navbar sidebar (hamburger menu) had two issues:
1. Links were not vertically centered
2. Links had insufficient left padding, appearing "stuck" to the left edge

### Solution Implemented in `src/css/custom.css` (Lines ~902-952)

Added dedicated mobile navbar sidebar alignment section:

#### 1. Panel Container Reset (Lines ~902-907)

```css
.navbar-sidebar__item.menu,
.theme-layout-navbar-sidebar-panel {
  padding: 0 !important;
  margin: 0 !important;
}
```

**Purpose**: Removes default padding/margin to have full control

#### 2. Menu Link Alignment with Padding (Lines ~909-917)

```css
.navbar-sidebar__item.menu .menu__link,
.theme-layout-navbar-sidebar-panel .menu__link {
  display: flex !important;
  align-items: center !important;
  padding: 8px 24px !important;  /* Increased from 16px to 24px */
  min-height: 40px !important;
  line-height: 1.5 !important;
}
```

**Key Changes**:
- `display: flex` with `align-items: center` - Vertical centering
- `padding: 8px 24px` - Increased horizontal padding from 16px to 24px
- `min-height: 40px` - Consistent touch target size for mobile

#### 3. Collapsible Items Alignment (Lines ~919-931)

```css
.navbar-sidebar__item.menu .menu__list-item-collapsible {
  margin: 0 !important;
}

.navbar-sidebar__item.menu .menu__list-item-collapsible .menu__link {
  display: flex !important;
  align-items: center !important;
  justify-content: space-between !important;
  padding: 8px 24px !important;  /* Increased to 24px */
}
```

**Purpose**:
- Aligns text on left and chevron on right
- Same 24px horizontal padding for consistency

#### 4. Icon Alignment (Lines ~933-940)

```css
.navbar-sidebar__item.menu .menu__link svg,
.theme-layout-navbar-sidebar-panel .menu__link svg {
  vertical-align: middle !important;
  display: inline-flex !important;
  align-items: center !important;
  flex-shrink: 0 !important;
}
```

**Purpose**: Ensures icons (chevrons, arrows) are vertically aligned with text

#### 5. Nested Items Indentation (Lines ~942-952)

```css
.navbar-sidebar__item.menu .menu__list .menu__list {
  padding-left: 12px !important;
  margin: 0 !important;
}

.navbar-sidebar__item.menu .menu__list .menu__list .menu__link {
  padding-left: 36px !important;  /* Increased from 28px to 36px */
}
```

**Purpose**:
- Proper indentation for nested menu items
- Increased from 28px to 36px to maintain spacing from left edge

### Padding Values Summary

**Top-level menu items**: 24px left/right (increased from 16px)
**Nested menu items**: 36px left padding (increased from 28px)
**Vertical padding**: 8px top/bottom (consistent)

### Benefits

1. **Better Touch Targets**: 40px minimum height for mobile usability
2. **Vertical Centering**: All text and icons properly aligned
3. **Adequate Spacing**: 24px left padding prevents "stuck to edge" appearance
4. **Consistent Indentation**: Nested items clearly hierarchical
5. **Icon Alignment**: Chevrons/arrows centered with text

### Mobile Sidebar Specific

This fix specifically targets:
- `.navbar-sidebar__item.menu` - Docusaurus mobile sidebar menu
- `.theme-layout-navbar-sidebar-panel` - Theme layout sidebar panel
- Only applies to mobile hamburger menu, not desktop sidebar

## Outcome

- ✅ Impact: Fixed mobile navbar sidebar alignment with proper padding and vertical centering
- 🧪 Tests: npm run build completed successfully (2 builds - initial fix and padding adjustment)
- 📁 Files: src/css/custom.css (added ~50 lines for mobile sidebar alignment)
- 🔁 Next prompts: Test on actual mobile device, verify touch target sizes, test nested menu behavior
- 🧠 Reflection: The fix uses specific selectors for the mobile navbar sidebar to avoid affecting the desktop sidebar. The increased padding (24px vs 16px) provides better visual spacing on mobile devices where screen real estate is limited but touch targets need adequate space.

## Evaluation notes (flywheel)

- Failure modes observed: None - build successful
- Graders run and results (PASS/FAIL): npm run build - PASS (2 successful builds)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test on various mobile device sizes (small phones, tablets), verify accessibility with screen readers on mobile
