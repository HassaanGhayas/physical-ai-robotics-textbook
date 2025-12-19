import { test, expect } from '@playwright/test';

test.describe('Mobile Responsiveness Tests', () => {
  test('should display correctly on iPhone SE (375px)', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('http://localhost:3000');

    // Check for horizontal scrollbar
    const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
    const viewportWidth = await page.evaluate(() => window.innerWidth);
    expect(bodyWidth).toBeLessThanOrEqual(viewportWidth);

    // Take screenshot
    await page.screenshot({ path: 'test-results/mobile-375px.png', fullPage: true });
  });

  test('should display correctly on iPhone 12 (390px)', async ({ page }) => {
    await page.setViewportSize({ width: 390, height: 844 });
    await page.goto('http://localhost:3000');

    // Check for horizontal scrollbar
    const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
    const viewportWidth = await page.evaluate(() => window.innerWidth);
    expect(bodyWidth).toBeLessThanOrEqual(viewportWidth);

    // Take screenshot
    await page.screenshot({ path: 'test-results/mobile-390px.png', fullPage: true });
  });

  test('should display correctly on iPad (768px)', async ({ page }) => {
    await page.setViewportSize({ width: 768, height: 1024 });
    await page.goto('http://localhost:3000');

    // Check for horizontal scrollbar
    const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
    const viewportWidth = await page.evaluate(() => window.innerWidth);
    expect(bodyWidth).toBeLessThanOrEqual(viewportWidth);

    // Take screenshot
    await page.screenshot({ path: 'test-results/tablet-768px.png', fullPage: true });
  });

  test('navbar should be responsive on mobile', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('http://localhost:3000');

    // Check if mobile menu button is visible
    const mobileMenuButton = page.locator('button[aria-label*="menu"], .navbar__toggle, [class*="toggle"]');
    await expect(mobileMenuButton.first()).toBeVisible();
  });
});
