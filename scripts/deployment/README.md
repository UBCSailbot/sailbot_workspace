# Deployment

Deploying our software to our autonomous sailboat's main computer.

## Deployment on the RaspberryPi and the Remote Server (Digital Ocean Droplet)

Comment out the website's docker compose file in the `devcontainer.json` to prevent the database from running.

```
	"dockerComposeFile": [
		"docker-compose.yml",
		// "website/docker-compose.website.yml", // website <- COMMENT OUT

		// Uncomment the files containing the programs you need
		// "docs/docker-compose.docs.yml", // docs
	],
```

Comment out the following extensions to prevent the raspberryPi from throttling in `devcontainer.json` as shown below.

```
			"extensions": [
				// Comment out the extensions when on the RPI
				// // template repo
				// "althack.ament-task-provider",
				// "DotJoshJohnson.xml",
				// "ms-azuretools.vscode-docker",
				// "ms-python.python",
				// "ms-vscode.cpptools",
				// "redhat.vscode-yaml",
				// "smilerobotics.urdf",
				// "streetsidesoftware.code-spell-checker",
				// "twxs.cmake",
				// "ms-python.flake8",
				// "yzhang.markdown-all-in-one",
				// // "zachflower.uncrustify",

				// // UBCSailbot
				// "awesomektvn.toggle-semicolon",
				// "bierner.github-markdown-preview",
				// "christian-kohler.path-intellisense",
				// "cschlosser.doxdocgen",
				// "DavidAnson.vscode-markdownlint",
				// "eamodio.gitlens",
				// // "github.copilot",
				// "github.vscode-github-actions",
				// "Gruntfuggly.todo-tree",
				// "jebbs.plantuml",
				// "jeff-hykin.better-cpp-syntax",
				// "KevinRose.vsc-python-indent",
				// "llvm-vs-code-extensions.vscode-clangd",
				// "matepek.vscode-catch2-test-adapter",
				// "mechatroner.rainbow-csv",
				// "mongodb.mongodb-vscode",
				// "ms-iot.vscode-ros",
				// "ms-python.black-formatter",
				// "ms-python.isort",
				// "ms-python.vscode-pylance",
				// "ms-toolsai.jupyter",
				// "ms-vsliveshare.vsliveshare",
				// "ms-python.mypy-type-checker",
				// "njpwerner.autodocstring",
				// "randomfractalsinc.geo-data-viewer",
				// "stevejpurves.cucumber",
				// "streetsidesoftware.code-spell-checker",
				// "vscode-icons-team.vscode-icons",
				// "zxh404.vscode-proto3"
```

## Scripts

### `setup_boot.sh`

Configures programs and scripts that need to run when the main computer boots. Only needs to be run once unless the
script is updated. Does not need to be rerun if any scripts or programs it targets are updated, with the exception of
renaming or moving the file.

Usage:

- Must be run as root: `sudo ./setup_boot.sh`

### `start_containers.sh`

Runs our Docker Compose files. You may have to install commands like `wget`.
Would recommend running this script in its own clone of sailbot_workspace (not the one you open in VS Code).

Usage:

- Runs the global launch file by default: `./start_containers.sh`
- Add the `--website` argument to additionally run the website container
- Add the `--interactive` argument to manually run commands in the sailbot workspace container
- Add the `--help` argument to see all available arguments
