import { test, expect } from '@playwright/test';

test.describe('Card Component Mobile Responsiveness', () => {
  test('HardwareSpecs card should not break on iPhone SE (375px)', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('http://localhost:3000/docs/book/hardware-requirements');

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Check for horizontal scrollbar
    const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
    const viewportWidth = await page.evaluate(() => window.innerWidth);
    expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1); // +1 for rounding

    // Find any card elements
    const cards = page.locator('[class*="card"]');
    const cardCount = await cards.count();

    if (cardCount > 0) {
      // Check first card doesn't overflow
      const firstCard = cards.first();
      const cardBox = await firstCard.boundingBox();

      if (cardBox) {
        expect(cardBox.width).toBeLessThanOrEqual(viewportWidth);
      }
    }

    // Take screenshot for visual verification
    await page.screenshot({ path: 'test-results/card-mobile-375px.png', fullPage: true });
  });

  test('HardwareSpecs card should not break on very small screen (320px)', async ({ page }) => {
    await page.setViewportSize({ width: 320, height: 568 });
    await page.goto('http://localhost:3000/docs/book/hardware-requirements');

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Check for horizontal scrollbar
    const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
    const viewportWidth = await page.evaluate(() => window.innerWidth);
    expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1);

    // Take screenshot
    await page.screenshot({ path: 'test-results/card-mobile-320px.png', fullPage: true });
  });

  test('Card text should wrap properly on mobile', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('http://localhost:3000/docs/book/hardware-requirements');

    await page.waitForLoadState('networkidle');

    // Check that card content uses word-wrap
    const cardStyles = await page.evaluate(() => {
      const card = document.querySelector('[class*="card"]');
      if (!card) return null;

      const computedStyles = window.getComputedStyle(card);
      return {
        overflowWrap: computedStyles.overflowWrap,
        wordWrap: computedStyles.wordWrap,
        minWidth: computedStyles.minWidth,
        maxWidth: computedStyles.maxWidth,
      };
    });

    if (cardStyles) {
      expect(cardStyles.overflowWrap).toBe('break-word');
      expect(cardStyles.maxWidth).toContain('%');
    }
  });
});
