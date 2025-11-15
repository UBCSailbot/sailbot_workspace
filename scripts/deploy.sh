#!/bin/bash

set -e

function helpMessage() {
    echo -e "Build release image with binaries and push to our GitHub Container Registry."
    echo -e "Usage: ./build.sh <GITHUB_PAT> <GITHUB_USERNAME> [FLAGS] ..."
    echo -e "Options (All Optional):"
    echo -e "\t-t <TAG>: Specifies a custom tag. Defaults to latest."
    echo -e "\t-h: Display this message."
}

# GitHub personal access token and username
PAT="$1"
USERNAME="$2"
TAG=latest

# Check for mandatory script arguments
if [ $# -lt 2 ]; then
  echo "Error: Missing required arguments."
  echo "Usage: $0 <arg1> <arg2>"
  exit 1
fi

shift 2

# Parse command-line options (all are optional arguments)
while getopts "t:h" flags; do
  case ${flags} in
    t) TAG=${OPTARG};;
    h) helpMessage; exit 0 ;;
    \? )    echo "Invalid option: -${flag}"; helpMessage; exit 1 ;;
    : )     echo "Option -${flag} requires an argument"; helpMessage; exit 1 ;;
  esac
done

# Login to our GitHub Container Registry
echo $PAT | docker login ghcr.io -u $USERNAME --password-stdin

# Check if Docker buildx drivers are present
if ! docker buildx inspect sailbot >/dev/null 2>&1; then
  docker buildx create --name sailbot --platform linux/arm64
fi

# Build release image
docker buildx build . \
    --file .devcontainer/release/release.Dockerfile \
    --tag ghcr.io/ubcsailbot/sailbot_workspace/release:${TAG} \
    --platform linux/arm64 \
    --builder sailbot \
    --build-arg CACHEBUST=$(date +%s%3N) \
    --push
