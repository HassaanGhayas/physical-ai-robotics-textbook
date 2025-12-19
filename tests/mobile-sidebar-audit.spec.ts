import { test, expect } from '@playwright/test';
import * as fs from 'fs';
import * as path from 'path';

/**
 * Mobile Sidebar UI/UX Audit Test Suite
 * Tests the mobile sidebar at multiple viewport dimensions to identify UI/UX issues
 */

const TEST_URL = 'http://localhost:3000/physical-ai-robotics-textbook/docs/book/hardware-requirements';
const SCREENSHOTS_DIR = path.join(__dirname, '../test-results/mobile-sidebar-audit');

// Ensure screenshots directory exists
if (!fs.existsSync(SCREENSHOTS_DIR)) {
  fs.mkdirSync(SCREENSHOTS_DIR, { recursive: true });
}

const VIEWPORTS = [
  { name: 'iPhone_SE', width: 375, height: 667, description: 'iPhone SE' },
  { name: 'iPhone_12', width: 390, height: 844, description: 'iPhone 12/13/14' },
  { name: 'iPhone_12_Pro_Max', width: 414, height: 896, description: 'iPhone 12 Pro Max' },
  { name: 'iPad_Portrait', width: 768, height: 1024, description: 'iPad Portrait' },
  { name: 'iPad_Landscape', width: 1024, height: 768, description: 'iPad Landscape' },
];

const AUDIT_ISSUES = {
  alignment: [] as string[],
  spacing: [] as string[],
  overflow: [] as string[],
  touchTargets: [] as string[],
  theme: [] as string[],
  other: [] as string[],
};

test.describe('Mobile Sidebar UI/UX Audit', () => {

  for (const viewport of VIEWPORTS) {
    test(`Test mobile sidebar at ${viewport.description} (${viewport.width}x${viewport.height})`, async ({ page }) => {

      // Set viewport
      await page.setViewportSize({
        width: viewport.width,
        height: viewport.height
      });

      console.log(`\n${'='.repeat(60)}`);
      console.log(`Testing: ${viewport.description} (${viewport.width}x${viewport.height}px)`);
      console.log('='.repeat(60));

      // Navigate to the page
      await page.goto(TEST_URL, { waitUntil: 'networkidle' });
      await page.waitForTimeout(1000); // Wait for animations

      // Take screenshot of initial state
      const initialScreenshot = path.join(SCREENSHOTS_DIR, `${viewport.name}_01_initial.png`);
      await page.screenshot({ path: initialScreenshot, fullPage: true });
      console.log(`✓ Initial screenshot saved: ${viewport.name}_01_initial.png`);

      // Find and click hamburger menu
      const hamburgerButton = page.locator('button[aria-label="Toggle navigation bar"]');
      await expect(hamburgerButton).toBeVisible();

      // Check hamburger button size (touch target)
      const hamburgerBox = await hamburgerButton.boundingBox();
      if (hamburgerBox) {
        console.log(`  Hamburger button size: ${Math.round(hamburgerBox.width)}x${Math.round(hamburgerBox.height)}px`);
        if (hamburgerBox.width < 44 || hamburgerBox.height < 44) {
          const issue = `${viewport.name}: Hamburger button too small (${Math.round(hamburgerBox.width)}x${Math.round(hamburgerBox.height)}px, should be >= 44x44px)`;
          AUDIT_ISSUES.touchTargets.push(issue);
          console.log(`  ⚠️  ${issue}`);
        } else {
          console.log('  ✓ Hamburger button touch target OK');
        }
      }

      // Click hamburger to open sidebar
      await hamburgerButton.click();
      await page.waitForTimeout(500); // Wait for sidebar animation

      // Take screenshot with sidebar open
      const sidebarOpenScreenshot = path.join(SCREENSHOTS_DIR, `${viewport.name}_02_sidebar_open.png`);
      await page.screenshot({ path: sidebarOpenScreenshot, fullPage: true });
      console.log(`✓ Sidebar open screenshot saved: ${viewport.name}_02_sidebar_open.png`);

      // Check if sidebar is visible
      const sidebar = page.locator('.navbar-sidebar');
      await expect(sidebar).toBeVisible();
      console.log('  ✓ Sidebar is visible');

      // Get sidebar dimensions
      const sidebarBox = await sidebar.boundingBox();
      if (sidebarBox) {
        console.log(`  Sidebar dimensions: ${Math.round(sidebarBox.width)}x${Math.round(sidebarBox.height)}px`);
        console.log(`  Sidebar position: x=${Math.round(sidebarBox.x)}, y=${Math.round(sidebarBox.y)}`);

        // Check if sidebar exceeds viewport width
        if (sidebarBox.width > viewport.width) {
          const issue = `${viewport.name}: Sidebar width (${Math.round(sidebarBox.width)}px) exceeds viewport width (${viewport.width}px)`;
          AUDIT_ISSUES.overflow.push(issue);
          console.log(`  ⚠️  ${issue}`);
        } else {
          console.log('  ✓ Sidebar width OK');
        }

        // Check if sidebar is properly positioned (should start from right edge)
        const expectedRight = viewport.width;
        const actualRight = sidebarBox.x + sidebarBox.width;
        const rightOffset = Math.abs(expectedRight - actualRight);
        if (rightOffset > 5) {
          const issue = `${viewport.name}: Sidebar not aligned to right edge (offset: ${Math.round(rightOffset)}px)`;
          AUDIT_ISSUES.alignment.push(issue);
          console.log(`  ⚠️  ${issue}`);
        } else {
          console.log('  ✓ Sidebar alignment OK');
        }
      }

      // Check sidebar header elements
      const sidebarBrand = sidebar.locator('.navbar-sidebar__brand');
      await expect(sidebarBrand).toBeVisible();

      const sidebarBrandBox = await sidebarBrand.boundingBox();
      if (sidebarBrandBox) {
        console.log(`  Sidebar header height: ${Math.round(sidebarBrandBox.height)}px`);
      }

      // Check close button
      const closeButton = sidebar.locator('.navbar-sidebar__close');
      await expect(closeButton).toBeVisible();
      console.log('  ✓ Close button is visible');

      const closeButtonBox = await closeButton.boundingBox();
      if (closeButtonBox) {
        console.log(`  Close button size: ${Math.round(closeButtonBox.width)}x${Math.round(closeButtonBox.height)}px`);
        if (closeButtonBox.width < 44 || closeButtonBox.height < 44) {
          const issue = `${viewport.name}: Close button too small (${Math.round(closeButtonBox.width)}x${Math.round(closeButtonBox.height)}px, should be >= 44x44px)`;
          AUDIT_ISSUES.touchTargets.push(issue);
          console.log(`  ⚠️  ${issue}`);
        } else {
          console.log('  ✓ Close button touch target OK');
        }
      }

      // Check sidebar menu links
      const menuLinks = sidebar.locator('.menu__link');
      const menuLinksCount = await menuLinks.count();
      console.log(`  Found ${menuLinksCount} menu links`);

      // Check first few menu links for touch target size
      const linksToCheck = Math.min(menuLinksCount, 3);
      for (let i = 0; i < linksToCheck; i++) {
        const link = menuLinks.nth(i);
        const linkBox = await link.boundingBox();
        if (linkBox && linkBox.height < 44) {
          const issue = `${viewport.name}: Menu link ${i + 1} too small (height: ${Math.round(linkBox.height)}px, should be >= 44px)`;
          AUDIT_ISSUES.touchTargets.push(issue);
          console.log(`  ⚠️  ${issue}`);
        }
      }

      // Check for horizontal scroll in sidebar
      const hasHorizontalScroll = await page.evaluate(() => {
        const sidebar = document.querySelector('.navbar-sidebar');
        if (sidebar) {
          return sidebar.scrollWidth > sidebar.clientWidth;
        }
        return false;
      });

      if (hasHorizontalScroll) {
        const issue = `${viewport.name}: Sidebar has horizontal scroll (content overflow)`;
        AUDIT_ISSUES.overflow.push(issue);
        console.log(`  ⚠️  ${issue}`);
      } else {
        console.log('  ✓ No horizontal scroll in sidebar');
      }

      // Check sidebar content spacing
      const sidebarItems = sidebar.locator('.navbar-sidebar__items');
      const sidebarItemsBox = await sidebarItems.boundingBox();
      if (sidebarItemsBox) {
        console.log(`  Sidebar content padding: checking...`);
        const computedStyle = await sidebarItems.evaluate((el) => {
          const style = window.getComputedStyle(el);
          return {
            paddingLeft: style.paddingLeft,
            paddingRight: style.paddingRight,
            paddingTop: style.paddingTop,
            paddingBottom: style.paddingBottom,
          };
        });
        console.log(`    Left: ${computedStyle.paddingLeft}, Right: ${computedStyle.paddingRight}`);
        console.log(`    Top: ${computedStyle.paddingTop}, Bottom: ${computedStyle.paddingBottom}`);
      }

      // Take a screenshot highlighting the close button
      await closeButton.highlight();
      await page.waitForTimeout(200);
      const closeButtonHighlight = path.join(SCREENSHOTS_DIR, `${viewport.name}_03_close_button.png`);
      await page.screenshot({ path: closeButtonHighlight, fullPage: true });
      console.log(`✓ Close button highlight saved: ${viewport.name}_03_close_button.png`);

      // Click close button to close sidebar
      await closeButton.click();
      await page.waitForTimeout(500); // Wait for sidebar animation

      // Verify sidebar is closed
      await expect(sidebar).not.toBeVisible();
      console.log('  ✓ Sidebar closes successfully');

      console.log(`✓ Test completed for ${viewport.description}\n`);
    });
  }

  test.afterAll(async () => {
    // Generate audit report
    console.log('\n' + '='.repeat(60));
    console.log('MOBILE SIDEBAR UI/UX AUDIT REPORT');
    console.log('='.repeat(60) + '\n');

    const reportPath = path.join(SCREENSHOTS_DIR, 'audit-report.txt');
    let report = 'MOBILE SIDEBAR UI/UX AUDIT REPORT\n';
    report += '='.repeat(60) + '\n\n';
    report += `Test Date: ${new Date().toISOString()}\n`;
    report += `Test URL: ${TEST_URL}\n\n`;

    let totalIssues = 0;

    const sections = [
      { title: 'Alignment Issues', issues: AUDIT_ISSUES.alignment },
      { title: 'Spacing Issues', issues: AUDIT_ISSUES.spacing },
      { title: 'Overflow Issues', issues: AUDIT_ISSUES.overflow },
      { title: 'Touch Target Issues (< 44px)', issues: AUDIT_ISSUES.touchTargets },
      { title: 'Theme Issues', issues: AUDIT_ISSUES.theme },
      { title: 'Other Issues', issues: AUDIT_ISSUES.other },
    ];

    for (const section of sections) {
      if (section.issues.length > 0) {
        report += `${section.title} (${section.issues.length}):\n`;
        report += '-'.repeat(60) + '\n';
        section.issues.forEach((issue, index) => {
          report += `${index + 1}. ${issue}\n`;
          console.log(`${section.title}: ${issue}`);
        });
        report += '\n';
        totalIssues += section.issues.length;
      }
    }

    if (totalIssues === 0) {
      report += 'No critical issues found! ✓\n';
      console.log('\n✓ No critical issues found!\n');
    } else {
      report += `Total Issues Found: ${totalIssues}\n`;
      console.log(`\nTotal Issues Found: ${totalIssues}\n`);
    }

    report += '\n' + '='.repeat(60) + '\n';
    report += 'Screenshots saved in: ' + SCREENSHOTS_DIR + '\n';
    report += '='.repeat(60) + '\n';

    fs.writeFileSync(reportPath, report);
    console.log(`Audit report saved to: ${reportPath}\n`);
  });
});
