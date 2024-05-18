# Docs Site

UBCSailbot software team's documentation site. It is meant to be developed in [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace){target=_blank}
in conjunction with our other software, but doesn't have to be. There are instructions for both cases below.

## Setup

### Setup in Sailbot Workspace

1. Uncomment `docs/docker-compose.docs.yml` in `.devcontainer/devcontainer.json`
2. Uncomment `8000:8000` in `.devcontainer/docker-compose.yml`
3. Rebuild the Dev Container

Refer to [How to work with containerized applications](../sailbot_workspace/usage/how_to.md#work-with-containerized-applications)
for more details.

### Setup Standalone

1. Manually install [social plugin OS dependencies](https://squidfunk.github.io/mkdocs-material/setup/setting-up-social-cards/#dependencies){target=_blank}

2. Install Python dependencies

    ```
   pip install --upgrade pip
   pip install -Ur docs/requirements.txt
   ```

    - Can do this in a [Python virtual environment](../../reference/python/virtual-environments.md)

## Run

### Run in Sailbot Workspace

After [setup](#setup-in-sailbot-workspace), the Docs site should be running on port 8000.

Refer to [How to work with containerized applications](../sailbot_workspace/usage/how_to.md#work-with-containerized-applications)
for more details.

### Run Standalone

```
mkdocs serve
```

## Update Dependencies

This site is built using the latest versions of dependencies in `docs/requirements.txt`
at the time of the most recent commit to the main branch.
To see exactly how the site will look when deployed, ensure your local dependencies are up to date.

### Update Dependencies in Sailbot Workspace

Rebuild the Dev Container.

### Update Dependencies By Itself

```
pip install -Ur docs/requirements.txt
```

## Maintain

### Delete Deployed Versions

A version of the docs site is created when a PR is open, and is deleted when it is merged or closed.
However, the CI that does this is very finnicky, so if 2 PR's are trying to update the site at the exact same time
one might fail. This is especially annoying if this happens to be one that deletes a version, because this means that
there is a version still open for a merged/closed PR. To manually clean up these PR's, run the following commands in
the docs container (in Docker Desktop, the exec tab):

```
git config user.name <your github username>
git config user.email <your github email>
mike delete --push pr-<number>
```

If you get an error that your local copy of the `gh-pages` branch has diverged from the remote, you can delete it
with `git branch -D gh-pages` and rerun the `mike delete` command above.

It will probably ask you to login to GitHub: enter your username then a [GitHub access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens)
with write permission.
