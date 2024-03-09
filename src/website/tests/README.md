# Tests

The tests for the website use [Cucumber.js](https://github.com/cucumber/cucumber-js), with
[Typescript](https://www.typescriptlang.org/).

## Prerequisites

- website is running as written in the README.md
- test config is set appropriately `.../tests/shared/config.ts`
- execute `npm install` from this directory to install dependencies.

## Scripts

| Command                      | Description            |
| ---------------------------- | ---------------------- |
| npm test                     | Runs all tests         |
| npm run test-tag @{tag-name} | Runs tests for TAGNAME |

### Writing Tests Help/Tips

- Common test steps -> `./steps/common.ts`
- All the CRUD API requests -> `./shared/classes/api.ts`
- If you expect requests to fail ensure `failOnError` is set to false on API request
- world object `./world/world.ts` defines the global `this` reference inside your steps.
