import { test, expect, Page } from '@playwright/test';

// The map filters only render in the Map pane; the summary only in the Stats
// pane. On mobile exactly one pane is shown (the other is display:none), so
// these two strings tell us which view is active.
const MAP_MARKER = 'AIS Ships';
const DATA_MARKER = 'POLARIS IS CURRENTLY';

const hasHorizontalOverflow = (page: Page) =>
  page.evaluate(
    () => document.documentElement.scrollWidth > window.innerWidth + 1,
  );

async function openNav(page: Page) {
  await page.getByRole('button', { name: 'Open navigation' }).click();
  await expect(page.getByRole('navigation')).toBeVisible();
}

test.describe('mobile: dashboard', () => {
  test('shows the mobile bar, hides the desktop header, no horizontal scroll', async ({
    page,
  }) => {
    await page.goto('/');
    await expect(
      page.getByRole('button', { name: 'Open navigation' }),
    ).toBeVisible();
    await expect(
      page.getByRole('heading', { name: 'SAILBOTPOLARIS.COM' }),
    ).toBeHidden();
    expect(await hasHorizontalOverflow(page)).toBe(false);
    // Map is the default pane.
    await expect(page.getByText(MAP_MARKER)).toBeVisible();
    await expect(page.getByText(DATA_MARKER)).toBeHidden();
  });

  test('nav lists MAP/DATA/ABOUT/DOWNLOAD and switches map <-> data', async ({
    page,
  }) => {
    await page.goto('/');
    await openNav(page);
    const nav = page.getByRole('navigation');
    for (const label of ['MAP', 'DATA', 'ABOUT', 'DOWNLOAD']) {
      await expect(nav.getByText(label, { exact: true })).toBeVisible();
    }

    await nav.getByText('DATA', { exact: true }).click();
    await expect(page.getByText(DATA_MARKER)).toBeVisible();
    await expect(page.getByText(MAP_MARKER)).toBeHidden();

    await openNav(page);
    await page
      .getByRole('navigation')
      .getByText('MAP', { exact: true })
      .click();
    await expect(page.getByText(MAP_MARKER)).toBeVisible();
    await expect(page.getByText(DATA_MARKER)).toBeHidden();
  });
});

test.describe('mobile: about', () => {
  test('no horizontal scroll, mobile nav present, pinned ABOUT US header', async ({
    page,
  }) => {
    await page.goto('/about');
    await expect(
      page.getByRole('button', { name: 'Open navigation' }),
    ).toBeVisible();
    await expect(
      page.getByRole('heading', { name: 'SAILBOTPOLARIS.COM' }),
    ).toBeHidden();
    expect(await hasHorizontalOverflow(page)).toBe(false);
    await expect(page.getByRole('heading', { name: 'ABOUT US' })).toBeVisible();
  });

  test('DATA from About deep-links to the dashboard data pane', async ({
    page,
  }) => {
    await page.goto('/about');
    await openNav(page);
    await page
      .getByRole('navigation')
      .getByText('DATA', { exact: true })
      .click();
    await expect(page).toHaveURL(/#data/);
    await expect(page.getByText(DATA_MARKER)).toBeVisible();
    await expect(page.getByText(MAP_MARKER)).toBeHidden();
  });
});

test.describe('mobile: download', () => {
  test('reachable from the nav, no horizontal scroll', async ({ page }) => {
    await page.goto('/');
    await openNav(page);
    await page
      .getByRole('navigation')
      .getByText('DOWNLOAD', { exact: true })
      .click();
    await expect(page).toHaveURL(/\/download$/);
    await expect(
      page.getByRole('heading', { name: 'DOWNLOAD OUR DATASETS' }),
    ).toBeVisible();
    expect(await hasHorizontalOverflow(page)).toBe(false);
  });
});
