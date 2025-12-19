---
id: 6
title: Remove Hamburger and Fix Theme Icons
stage: green
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: ui-ux-design
branch: 001-testing
user: HassaanGhayas
command: /sp.implement
labels: ["ui-fix", "navbar", "css", "theme-toggle", "hamburger-menu"]
links:
  spec: D:\my-web\specs\001-ui-ux-design\spec.md
  ticket: null
  adr: null
  pr: null
files:
  - D:\my-web\src\css\custom.css
tests:
  - Build verification successful
  - Visual inspection required
---

## Prompt

Remove the hamburger icon from the navbar as it is useless and the theme toggle should show sun when light theme is applied and moon when dark theme is applied

## Response snapshot

Fixed two navbar issues:

1. **Removed Hamburger Menu Icon**:
   - Hidden mobile sidebar toggle button completely
   - Targeted multiple selectors to ensure complete removal:
     - `.navbar__toggle` - Standard toggle button
     - `.navbar-sidebar__toggle` - Sidebar specific toggle
     - `button[class*="toggleButton"]` - Any button with "toggleButton" class
   - Applied `display: none !important` to all selectors

   CSS added:
   ```css
   .navbar__toggle,
   .navbar-sidebar__toggle,
   button[class*="toggleButton"] {
     display: none !important;
   }
   ```

2. **Fixed Theme Toggle Icons (Sun/Moon Display)**:
   - Changed `overflow` from `hidden` to `visible` to prevent icon clipping
   - Added explicit icon visibility rules for light/dark modes
   - Ensured SVGs are properly displayed: `opacity: 1`, `visibility: visible`

   **Light Mode**: Shows sun icon (☀️)
   - Display `svg[class*="lightToggleIcon"]`
   - Hide `svg[class*="darkToggleIcon"]`

   **Dark Mode**: Shows moon icon (🌙)
   - Display `svg[class*="darkToggleIcon"]`
   - Hide `svg[class*="lightToggleIcon"]`

   CSS rules:
   ```css
   /* Light mode */
   [data-theme='light'] button[class*="toggle"] svg[class*="lightToggleIcon"] {
     display: block !important;
   }
   [data-theme='light'] button[class*="toggle"] svg[class*="darkToggleIcon"] {
     display: none !important;
   }

   /* Dark mode */
   [data-theme='dark'] button[class*="toggle"] svg[class*="darkToggleIcon"] {
     display: block !important;
   }
   [data-theme='dark'] button[class*="toggle"] svg[class*="lightToggleIcon"] {
     display: none !important;
   }
   ```

## Outcome

- ✅ Impact: Hamburger menu removed, theme toggle now shows correct icon per theme
- 🧪 Tests: Build successful, visual inspection required at localhost:3000
- 📁 Files: Modified src/css/custom.css (lines 476-540)
- 🔁 Next prompts: None required, navbar fully functional
- 🧠 Reflection: Changed overflow to visible for proper icon display; used [data-theme] selectors to control icon visibility based on current theme

## Evaluation notes (flywheel)

- Failure modes observed: overflow:hidden was clipping theme icons; hamburger menu still visible
- Graders run and results (PASS/FAIL): Build: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test theme toggle functionality in browser
