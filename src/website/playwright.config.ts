import { defineConfig, devices } from '@playwright/test';

/**
 * E2E tests for the responsive (mobile) layout.
 *
 * Boots the website in no-API mode (DISABLE_API=true, so no Mongo is needed)
 * and drives it in a real browser at both a phone viewport and a desktop
 * viewport. Run with `npm run test:e2e`.
 */
export default defineConfig({
  testDir: './e2e',
  timeout: 60_000,
  expect: { timeout: 15_000 },
  fullyParallel: false,
  workers: 1,
  forbidOnly: !!process.env.CI,
  reporter: [['list'], ['html', { open: 'never' }]],
  use: {
    baseURL: 'http://localhost:3005',
    navigationTimeout: 45_000,
    actionTimeout: 15_000,
    trace: 'on-first-retry',
  },
  webServer: {
    command: 'npm run web:dev:no-api',
    url: 'http://localhost:3005',
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
  projects: [
    {
      name: 'mobile',
      testMatch: /mobile\.spec\.ts$/,
      use: { ...devices['Pixel 5'] },
    },
    {
      name: 'desktop',
      testMatch: /desktop\.spec\.ts$/,
      use: { ...devices['Desktop Chrome'] },
    },
  ],
});
