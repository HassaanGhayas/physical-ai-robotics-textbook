---
title: Accessibility Guidelines
sidebar_label: Accessibility
description: Accessibility guidelines and features for the Physical AI & Humanoid Robotics book
---

# Accessibility Guidelines

The Physical AI & Humanoid Robotics book follows accessibility best practices to ensure content is available to all users, including those with disabilities.

## Color Contrast

All text meets WCAG AA contrast standards (4.5:1 for normal text, 3:1 for large text). The theme uses carefully selected colors that maintain good contrast in both light and dark modes.

## Keyboard Navigation

All interactive elements are accessible via keyboard:
- Use Tab to navigate between focusable elements
- Use Enter or Space to activate buttons and links
- Use arrow keys to navigate through components like navigation menus

## Screen Reader Support

The site includes proper semantic HTML and ARIA attributes:
- All images have descriptive alt text
- Headings follow proper hierarchy (H1, H2, H3, etc.)
- Interactive elements have clear labels
- Navigation landmarks are properly marked

## Component Accessibility

### Code Examples
- Code blocks include syntax highlighting for better visual parsing
- Copy buttons have clear labels for screen readers
- Expandable code sections are clearly announced

### Hardware Specifications
- Tables use proper header associations
- Pro/con lists are structured as proper lists
- Cost information is clearly labeled

### Technical Diagrams
- Images include descriptive alt text
- Zoom functionality is keyboard accessible
- Captions provide additional context

## Responsive Design

The site is fully responsive and works well on:
- Mobile devices (320px and up)
- Tablets (768px and up)
- Desktops (1024px and up)

## Customization Options

### Theme Switching
Users can choose between light, dark, and auto themes using the ThemeSwitcher component.

### Text Scaling
The site respects user's system text scaling preferences and maintains readability at larger font sizes.

## Testing

The accessibility of the site has been tested with:
- Screen readers (NVDA, JAWS, VoiceOver)
- Keyboard-only navigation
- Color contrast analyzers
- Responsive design testing tools