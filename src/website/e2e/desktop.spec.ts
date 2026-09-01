import { test, expect } from '@playwright/test';

// Guards against the mobile responsive work leaking into the desktop layout.
test.describe('desktop: dashboard', () => {
  test('desktop header shown, hamburger hidden, map + stats side by side', async ({
    page,
  }) => {
    await page.goto('/');
    await expect(
      page.getByRole('heading', { name: 'SAILBOTPOLARIS.COM' }),
    ).toBeVisible();
    await expect(
      page.getByRole('button', { name: 'Open navigation' }),
    ).toBeHidden();
    // Both panes are visible at the same time on desktop.
    await expect(page.getByText('AIS Ships')).toBeVisible();
    await expect(page.getByText('Temporary during voyage')).toBeVisible();
  });
});
