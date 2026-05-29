## Instructions to Deploy Our Software to the Raspberry Pi

1. Push your changes to your branch on GitHub and go to the Build Release
   Image workflow page on GitHub

2. Click run workflow change the branch from main to your branch, add a tag
   (example-tag) and click run workflow

3. Obtain a classic personal access token (PAT) from GitHub if you do not
   already have one. It’s a more secure way to log in to GitHub

4. Once the workflow is complete (after 4-5 minutes) connect to the raspberry
   pi using SSH and navigate to the home directory: `~/`

5. Run the following:

   ```bash
   export CR_PAT=<YOUR_GITHUB_PERSONAL_ACCESS_TOKEN>

   echo $CR_PAT | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin

   docker pull ghcr.io/ubcsailbot/sailbot_workspace/release:example-tag

   # This step of renaming the image is optional but it will give the image
   # a tidier name :)
   docker tag ghcr.io/ubcsailbot/sailbot_workspace/release:example-tag release

   # this starts the container and drops you right into a bash shell inside
   # the container; you can exit by running: exit
   docker run -it --network host --privileged release bash
   ```

## Instructions for local deployment onto the Raspberry Pi

1. To build and save the release image locally,
navigate to repository's root directory `sailbot_workspace/`
and run the following on your machine. Make sure to change (example-tag) accordingly.

   ```bash
   docker build \
   --platform linux/arm64 \
   -f .devcontainer/release/release.Dockerfile \`
   --build-arg CACHEBUST=$(date +%s%3N) \
   -t release:example-tag \
   .
   
   # Saves the release image as a .tar file to then transfer via rsync.
   docker save -o release.tar release:example-tag
   ```

2. Connect to the `raye_wifi` network and connect to the raspberry pi using SSH.
Alternatively, you can remote SSH into the raspberry pi. Note that these
IP addresses are for the raspberry pi that's onboard the boat.

3. Transfer the release image tar file by running the following on your machine.

   ```bash
   rsync -a release.tar sailbot@192.168.0.10:/home/sailbot/
   ```

   If you are using remote SSH, run the following:

   ``` bash
   rysnc -a release.tar sailbot@100.95.219.39:/home/sailbot/
   ```

4. Load the release image and start the container on the rpi.
Run the following on the raspberry pi itself.

   ```bash
   docker load -i release.tar

   # this starts the container and drops you right into a bash shell inside
   # the container; you can exit by running: exit
   docker run -it --network host --privileged release:example-tag bash

   ```
