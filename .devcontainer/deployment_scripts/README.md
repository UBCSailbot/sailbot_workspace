# Deployment Scripts

## start_container.sh

- [Default]  Builds and runs the latest base docker image in sailbot_workspace
- [Optional] Run with a docker image ID as the first argument to run a specific image

```
./start_container.sh
./start_container.sh <IMAGE_ID>
```

## quick_build.sh

- Builds all Sailbot software code without checks that slow down the build like static analysis

```
./quick_build.sh
```
