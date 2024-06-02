# Deployment

Deploying our software to our autonomous sailboat's main computer.

## Scripts

### `setup_boot.sh`

Configures programs and scripts that need to run when the main computer boots. Only needs to be run once unless the
script is updated. Does not need to be rerun if any scripts or programs it targets are updated, with the exception of
renaming or moving the file.

Usage:

- Must be run as root: `sudo ./setup_boot.sh`

### `start_containers.sh`

Runs our Docker Compose files.

Usage:

- Runs the global launch file by default: `./start_containers.sh`
- Add the `--website` argument to additionally run the website container
- Add the `--interactive` argument to manually run commands in the sailbot workspace container
- Add the `--help` argument to see all available arguments
