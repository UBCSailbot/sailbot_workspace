# Help

## Performance Issues

If you are not satisfied with the performance of Sailbot Workspace, here are some things you can try:

- Free up memory: close programs that you aren't using
- Free up disk space: permanently delete large programs and files that you don't need anymore
- Run Sailbot Workspace in a GitHub Codespace
    - In a codespace with 8GB of RAM, building all packages from scratch with the `-q` argument takes about a minute.
    If your computer takes longer than, or you want to free up memory and disk space, you can
    [setup Sailbot Workspace in a GitHub Codespace](./setup.md#setup-sailbot-workspace-in-a-github-codespace){target=_blank}
- If you are running Sailbot Workspace on Windows, dual boot Ubuntu and run Sailbot Workspace there
    - Sailbot Workspace performs worse on Windows than bare metal Linux because it uses Docker, which is not natively supported.
    - Here is a guide to dual boot the operating systems we recommend: [How to Dual Boot Ubuntu 22.04 LTS and Windows 11](https://www.linuxtechi.com/dual-boot-ubuntu-22-04-and-windows-11/){target=_blank}
        - We recommend allocating at least 50 GB to Ubuntu to leave some wiggle room for Docker
        - The process is similar for other Ubuntu and Windows versions,
          but feel free to search for a guide specific to the combination you want to dual boot
        - Since Sailbot Workspace uses Docker, it should be able to run on any Linux distribution, not just Ubuntu.
          However, we may not be able to provide support if you encounter any difficulties with this

## Troubleshooting

If you are having some trouble running our software, here are some things you can try:

### Sailbot Workspace Troubleshooting

- [Update Sailbot Workspace](./workflow.md#2-update-sailbot-workspace){target=_blank}
- Run the `setup` task to update package dependencies
- Build from scratch
    1. Run the `clean` task to delete C++ generated files
    2. Run the `purge` task to delete ROS generated files
    3. Run the `Build All` task to rebuild

### VS Code Troubleshooting

- Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` VS Code command
- Reload VS Code: run the `Developer: Reload Window` VS Code command
- Identify broken extension: run the `Help: Start Extension Bisect` VS Code command
    - Once you have identified a broken extension, you can [install a previous version](https://stackoverflow.com/a/53755378){target=_blank}
      until the issue is fixed in a new release

### System Troubleshooting

- Restart WSL: close Sailbot Workspace and Docker Desktop then run `wsl --shutdown` in PowerShell
- Restart computer

### Docker Troubleshooting

- Delete Docker files

    ??? tip "Running Docker CLI commands on Windows"

        On Windows, Docker CLI commands should be run in the Ubuntu terminal while Docker Desktop is running.

    - Run `docker system prune` to remove all unused containers, networks, and dangling and unreferenced images
        - Add `--all` to additionally remove unused images (don't have a container associated with them)
        - Add `--volumes` to additionally remove volumes (makes Bash history and ROS logs persist across containers)
    - Run `docker rmi -f $(docker images -aq)` to remove all images
- [Install a previous version of Docker Desktop](https://stackoverflow.com/a/77224786){target=_blank}
