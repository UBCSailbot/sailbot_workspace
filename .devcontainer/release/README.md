## Instructions to Deploy Our Software to the Raspberry Pi

1. Push your changes to your branch on GitHub and go to the Build Release Image workflow page on GitHub

2. Click run workflow change the branch from main to your branch, add a tag (example-tag) and click run workflow

3. Obtain a classic personal access token (PAT) from GitHub if you do not already have one. It’s a more secure way to log in to GitHub

4. Once the workflow is complete (after 4-5 minutes) connect to the raspberry pi using SSH and navigate to the home directory: `~/`

5. Run the following:
```bash
  export CR_PAT=<YOUR_GITHUB_PERSONAL_ACCESS_TOKEN>
  
  echo $CR_PAT | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin
  
  docker pull ghcr.io/ubcsailbot/sailbot_workspace/release:example-tag
  
  # This step of renaming the image is optional but it will give the image a tidier name :)
  docker tag ghcr.io/ubcsailbot/sailbot_workspace/release:example-tag release

  # this starts the container and drops you right into a bash shell inside the container
  # you can exit by running: exit
  docker run -it --network host --privileged release bash 
```
