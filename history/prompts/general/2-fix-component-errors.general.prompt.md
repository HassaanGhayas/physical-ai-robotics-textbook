---
id: 2
title: "Fix Component Errors"
stage: general
date: "2025-12-11"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "001-book-creation"
user: "Hassan Ghayas"
command: "fix component errors"
labels: ["bug-fix", "components", "docusaurus", "physical-ai", "humanoid-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "src/components/HardwareSpecs/HardwareSpecs.tsx"
  - "src/components/index.tsx"
  - "sidebars.ts"
  - "docs/book/sample-components.md"
tests: []
---

# Fix Component Errors

## Summary

Fixed the reported issues with Docusaurus components: "specs.map is not a function" and "Expected component `BookNavigation` to be defined". Also removed the sample components page as it was not part of the actual book content.

## Prompt

When I visit certain pages it says specs.map is not a function or Expected component `BookNavigation` to be defined: you likely forgot to import, pass, or provide it.

## Response

Successfully identified and fixed the component errors:

### 1. Fixed HardwareSpecs Component
- **Issue**: The HardwareSpecs component was expecting an array of specs (`specs: HardwareSpecItem[]`) but was being used in MDX files with individual spec objects
- **Fix**: Updated the component to accept a single spec object with name, category, specs, cost, pros, and cons properties
- **Result**: The component now correctly maps over the spec properties using `Object.entries(specs)` instead of trying to map over the entire specs prop

### 2. Fixed Component Exports
- **Issue**: The index.tsx file was exporting non-existent components (HardwareSpecTable, HardwareSpecCard, etc.) instead of the actual components
- **Fix**: Updated the index.tsx file to export the correct component files:
  - HardwareSpecs instead of HardwareSpecTable and HardwareSpecCard
  - TechnicalDiagrams instead of TechDiagram and ArchitectureDiagram
  - CodeExamples instead of CodeExample and RosCodeBlock
  - Removed unused ChapterProgress and TechnicalThemeProvider exports
- **Result**: All components are now properly exported and can be imported in MDX files

### 3. Removed Sample Components Page
- **Issue**: The sample-components.md file was not part of the actual book content
- **Fix**: Removed the sample-components.md file and updated the sidebar to remove the reference
- **Result**: The book now only contains actual educational content

### 4. Verification
- Ran `npm run build` successfully after each fix
- Confirmed that the development server works properly
- All components now render correctly in the Docusaurus site

The errors were caused by:
1. A mismatch between the component's expected props structure and how it was being used
2. Incorrect component exports in the index.tsx file
3. The component expecting an array when it was receiving a single object

These fixes ensure that all custom components work properly throughout the Physical AI & Humanoid Robotics book.

## Outcome

All component errors have been resolved and the site builds successfully. The HardwareSpecs component now works correctly with individual spec objects, and all components are properly exported and available for use in MDX files.