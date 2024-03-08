# Docs

[![Tests](https://github.com/UBCSailbot/docs/actions/workflows/tests.yml/badge.svg)](https://github.com/UBCSailbot/docs/actions/workflows/tests.yml)

UBCSailbot software team's documentation site. It is meant to be developed in [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace)
in conjunction with our other software, but doesn't have to be. There are instructions for both cases below.

## Setup

### Setup in Sailbot Workspace

1. Uncomment `docker-compose.docs.yml` in `.devcontainer/devcontainer.json`
2. Rebuild the Dev Container

Refer to [How to run optional programs](https://ubcsailbot.github.io/docs/main/current/sailbot_workspace/how_to/#run-optional-programs)
for more details.

### Setup By Itself

1. Clone repository

    ```
    git clone https://github.com/UBCSailbot/docs.git
    ```

2. Manually install [social plugin OS dependencies](https://squidfunk.github.io/mkdocs-material/setup/setting-up-social-cards/#dependencies)

3. Install Python dependencies

    ```
   pip install --upgrade pip
   pip install -Ur docs/requirements.txt
   ```

    - Can do this in a [Python virtual environment](https://ubcsailbot.github.io/docs/reference/python/virtual-environments/)

## Run

### Run in Sailbot Workspace

After [setup](#setup-in-sailbot-workspace), the Docs site should be running on port 8000.

Refer to [How to run optional programs](https://ubcsailbot.github.io/docs/main/current/sailbot_workspace/how_to/#run-optional-programs)
for more details.

### Run By Itself using VS Code

1. `CTRL+P` to open Quick Open
2. Run a launch configuration
    - "debug Run Application" runs `mkdocs serve`
    - "debug Launch Application" runs `mkdocs serve` and opens the application in a new Microsoft Edge window

### Run By Itself using CLI

```
mkdocs serve
```

## Update Dependencies

This site is built using the latest versions of dependencies in [`docs/requirements.txt`](./docs/requirements.txt)
at the time of the most recent commit to the main branch.
To see exactly how the site will look when deployed, ensure your local dependencies are up to date.

### Update Dependencies in Sailbot Workspace

Rebuild the Dev Container.

### Update Dependencies By Itself

```
pip install -Ur docs/requirements.txt
```
