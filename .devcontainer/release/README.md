## Instructions to deploy our software to the Raspberry Pi

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
   docker run --name example-name -it --network host --privileged release bash
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

   ```bash
   rysnc -a release.tar sailbot@100.95.219.39:/home/sailbot/
   ```

4. Load the release image and start the container on the rpi.
Run the following on the raspberry pi itself.

   ```bash
   docker load -i release.tar

   # this starts the container and drops you right into a bash shell inside
   # the container; you can exit by running: exit
   docker run \
   --name example-name \
   -it --network host --privileged \
   release:example-tag \
   bash

   ```

## Instructions for starting our software once deployed

1. After creating the release-container with `docker run ...`, we can
re-enter the container and start all the ROS nodes if we're SSHed into
the raspberry Pi.

2. To enter the container and immediately start our software in production,
run the following and change the container name (example-name) accordingly:

   ```bash
   docker start example-name && \
   docker exec -it example-name bash -ic "ros2 launch \
      src/global_launch/main_launch.py record:=true mode:=production \
      2>&1 | tee src/global_launch/voyage_log/combined_log_$(date %F_%T).txt"
   ```

- To enter the containter without starting all ROS nodes, run the following:

   ```bash
   docker start -i example-name
   ```

- To extract the log files from the release-container run the following:
<!-- markdownlint-disable MD013 -->
   ```bash
   docker cp example-name:/workspaces/sailbot_workspace/src/global_launch/voyage_log ./voyage_log
   ```

- To extract ros2bags from the release-container run the following:

   ```bash
   docker cp example-name:/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recording ./session_recording
   ```
