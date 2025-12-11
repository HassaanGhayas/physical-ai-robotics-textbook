---
title: Custom Components Guide
sidebar_label: Components Guide
description: Guide to using custom components in the Physical AI & Humanoid Robotics book
---

# Custom Components Guide

This guide explains how to use the custom components created for the Physical AI & Humanoid Robotics book.

## HardwareSpecs Component

The `HardwareSpecs` component displays hardware specifications in a structured format with pros, cons, and cost information.

### Usage

```md
import HardwareSpecs from '@site/src/components/HardwareSpecs/HardwareSpecs';

<HardwareSpecs
  name="Component Name"
  category="Category"
  specs={{
    cpu: "CPU specification",
    gpu: "GPU specification",
    memory: "Memory specification",
    storage: "Storage specification"
  }}
  cost={{
    price: "Price information",
    reasoning: "Reasoning for cost"
  }}
  pros={[
    "Advantage 1",
    "Advantage 2"
  ]}
  cons={[
    "Disadvantage 1",
    "Disadvantage 2"
  ]}
/>
```

## CodeExamples Component

The `CodeExamples` component displays code with syntax highlighting, copy functionality, and expand/collapse features.

### Usage

```md
import CodeExamples from '@site/src/components/CodeExamples/CodeExamples';

<CodeExamples
  title="Code Example Title"
  description="Description of the code example"
  language="python"  // or javascript, cpp, etc.
  code={`// Your code here
function example() {
  console.log("Hello, world!");
}`}
  copyable={true}
  expandable={true}
  maxHeight="300px"
/>
```

## TechnicalDiagrams Component

The `TechnicalDiagrams` component displays technical diagrams with zoom functionality and interactive controls.

### Usage

```md
import TechnicalDiagrams from '@site/src/components/TechnicalDiagrams/TechnicalDiagrams';

<TechnicalDiagrams
  title="Diagram Title"
  description="Description of the diagram"
  imageUrl="/path/to/image.png"
  caption="Caption for the image"
  altText="Alternative text for accessibility"
  interactive={true}
  zoomable={true}
/>
```

## BookNavigation Component

The `BookNavigation` component provides custom navigation for the book with progress tracking and bookmarking.

### Usage

```md
import BookNavigation from '@site/src/components/BookNavigation/BookNavigation';

<BookNavigation
  title="Navigation Title"
  items={[
    {id: "1", title: "Chapter 1", url: "/chapter1", level: 1},
    {id: "2", title: "Section 1.1", url: "/section1-1", level: 2}
  ]}
  currentId="1"
  showProgress={true}
  showBookmarks={true}
/>
```

## ThemeSwitcher Component

The `ThemeSwitcher` component allows users to switch between light, dark, and auto themes.

### Usage

```md
import ThemeSwitcher from '@site/src/components/ThemeSwitcher/ThemeSwitcher';

<ThemeSwitcher />
```