import { test, expect } from '@playwright/test';

/**
 * Comprehensive Responsive Design Tests
 * Minimum supported width: 320px (iPhone SE, older devices)
 * Tests verify NO horizontal scrolling and proper content adaptation
 */

const viewports = [
  { name: 'iPhone SE (smallest)', width: 320, height: 568 },
  { name: 'iPhone SE 2020', width: 375, height: 667 },
  { name: 'iPhone 12/13', width: 390, height: 844 },
  { name: 'iPhone 12/13 Pro Max', width: 428, height: 926 },
  { name: 'Samsung Galaxy S20', width: 360, height: 800 },
  { name: 'iPad Mini', width: 768, height: 1024 },
  { name: 'iPad Pro', width: 1024, height: 1366 },
];

const testPages = [
  { url: '/', name: 'Homepage' },
  { url: '/physical-ai-robotics-textbook', name: 'Textbook Main' },
  { url: '/physical-ai-robotics-textbook/introduction', name: 'Introduction' },
  { url: '/physical-ai-robotics-textbook/hardware-requirements', name: 'Hardware Requirements' },
];

test.describe('320px Minimum Width Responsive Design', () => {
  for (const viewport of viewports) {
    for (const page of testPages) {
      test(`${page.name} should not overflow at ${viewport.width}px (${viewport.name})`, async ({ page: browserPage }) => {
        await browserPage.setViewportSize({ width: viewport.width, height: viewport.height });
        await browserPage.goto(`http://localhost:3000${page.url}`);
        await browserPage.waitForLoadState('networkidle');

        // Critical test: Check for horizontal scrollbar
        const measurements = await browserPage.evaluate(() => {
          return {
            bodyWidth: document.body.scrollWidth,
            htmlWidth: document.documentElement.scrollWidth,
            viewportWidth: window.innerWidth,
            bodyOffsetWidth: document.body.offsetWidth,
          };
        });

        // Allow 1px tolerance for rounding
        expect(measurements.bodyWidth, 'Body should not exceed viewport width').toBeLessThanOrEqual(measurements.viewportWidth + 1);
        expect(measurements.htmlWidth, 'HTML should not exceed viewport width').toBeLessThanOrEqual(measurements.viewportWidth + 1);

        // Verify no visible elements overflow (exclude hidden/offscreen absolute positioned elements)
        const overflowingElements = await browserPage.evaluate(() => {
          const elements = Array.from(document.querySelectorAll('*'));
          const viewportWidth = window.innerWidth;
          const problematic: string[] = [];

          elements.forEach((el) => {
            const rect = el.getBoundingClientRect();
            const styles = window.getComputedStyle(el);

            // Skip elements that are:
            // - Absolutely positioned and offscreen (top < 0)
            // - Hidden (display: none, visibility: hidden)
            // - Skip links (intentionally offscreen until focused)
            const isOffscreenAbsolute = styles.position === 'absolute' && rect.top < 0;
            const isHidden = styles.display === 'none' || styles.visibility === 'hidden';
            const isSkipLink = (el as HTMLElement).classList.toString().includes('skip');

            if (isOffscreenAbsolute || isHidden || isSkipLink) {
              return;
            }

            // Check if visible element extends beyond viewport
            if (rect.right > viewportWidth + 1) {
              const tagName = el.tagName.toLowerCase();
              const className = el.className;
              const id = el.id;
              problematic.push(`${tagName}${id ? '#' + id : ''}${className ? '.' + className.split(' ')[0] : ''} (width: ${rect.width}px, right: ${rect.right}px)`);
            }
          });

          return problematic;
        });

        expect(overflowingElements, `No visible elements should overflow viewport at ${viewport.width}px`).toHaveLength(0);
      });
    }
  }
});

test.describe('Critical Component Tests at 320px', () => {
  test('Navigation should be usable at 320px', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/physical-ai-robotics-textbook');
    await page.waitForLoadState('networkidle');

    // Mobile menu button should be visible
    const mobileMenuButton = page.locator('button[aria-label*="menu"], .navbar__toggle, [class*="toggle"]').first();
    await expect(mobileMenuButton).toBeVisible();

    // Screenshot for verification
    await page.screenshot({ path: 'test-results/nav-320px.png' });
  });

  test('Cards should stack and wrap text at 320px', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/physical-ai-robotics-textbook');
    await page.waitForLoadState('networkidle');

    // Check card wrapping
    const cards = page.locator('[class*="card"]');
    const cardCount = await cards.count();

    if (cardCount > 0) {
      const firstCard = cards.first();
      const cardWidth = await firstCard.evaluate((el) => el.getBoundingClientRect().width);
      expect(cardWidth).toBeLessThanOrEqual(320);

      // Verify text wrapping is enabled
      const wrapStyles = await firstCard.evaluate((el) => {
        const styles = window.getComputedStyle(el);
        return {
          overflowWrap: styles.overflowWrap,
          wordWrap: styles.wordWrap,
          whiteSpace: styles.whiteSpace,
        };
      });

      expect(wrapStyles.overflowWrap).toBe('break-word');
    }
  });

  test('Images should scale down at 320px', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/physical-ai-robotics-textbook');
    await page.waitForLoadState('networkidle');

    // Check all images fit within viewport
    const oversizedImages = await page.evaluate(() => {
      const images = Array.from(document.querySelectorAll('img'));
      const viewportWidth = window.innerWidth;
      return images
        .filter(img => img.getBoundingClientRect().width > viewportWidth)
        .map(img => ({
          src: img.src,
          width: img.getBoundingClientRect().width,
        }));
    });

    expect(oversizedImages).toHaveLength(0);
  });

  test('Tables should be scrollable or responsive at 320px', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/physical-ai-robotics-textbook/hardware-requirements');
    await page.waitForLoadState('networkidle');

    // Tables should either fit or have horizontal scroll enabled on container
    const tables = page.locator('table');
    const tableCount = await tables.count();

    for (let i = 0; i < tableCount; i++) {
      const table = tables.nth(i);
      const tableContainer = await table.evaluate((el) => {
        // Find scrollable parent container
        let parent = el.parentElement;
        while (parent) {
          const styles = window.getComputedStyle(parent);
          if (styles.overflowX === 'auto' || styles.overflowX === 'scroll') {
            return {
              hasScroll: true,
              containerWidth: parent.getBoundingClientRect().width,
            };
          }
          parent = parent.parentElement;
        }
        return {
          hasScroll: false,
          tableWidth: el.getBoundingClientRect().width,
        };
      });

      // Either table fits OR it has scrollable container
      if (!tableContainer.hasScroll) {
        expect(tableContainer.tableWidth).toBeLessThanOrEqual(320);
      }
    }
  });

  test('Typography should scale appropriately at 320px', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/physical-ai-robotics-textbook');
    await page.waitForLoadState('networkidle');

    const fontSize = await page.evaluate(() => {
      const html = document.documentElement;
      return parseFloat(window.getComputedStyle(html).fontSize);
    });

    // At 320px, base font size should be 14px or less (from custom.css @media 480px)
    expect(fontSize).toBeLessThanOrEqual(14);
  });
});

test.describe('Visual Regression - Screenshots', () => {
  test('Generate full-page screenshots at all breakpoints', async ({ page }) => {
    const breakpoints = [
      { width: 320, name: 'xs' },
      { width: 375, name: 'sm' },
      { width: 768, name: 'md' },
      { width: 1024, name: 'lg' },
      { width: 1440, name: 'xl' },
    ];

    for (const bp of breakpoints) {
      await page.setViewportSize({ width: bp.width, height: 1000 });
      await page.goto('http://localhost:3000');
      await page.waitForLoadState('networkidle');
      await page.screenshot({
        path: `test-results/homepage-${bp.name}-${bp.width}px.png`,
        fullPage: true,
      });
    }
  });
});
