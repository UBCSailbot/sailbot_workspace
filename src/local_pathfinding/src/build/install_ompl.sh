#!/bin/bash
set -e

REPO="ompl/ompl"
RELEASE_TAG="1.7.0"
PYTHON_VERSION="cp310"
TMP_DIR="/tmp"

arch=$(uname -m)

if [ "$arch" = "x86_64" ]; then
    ZIP_ASSET="wheels-ubuntu-latest-x86_64.zip"
elif [ "$arch" = "aarch64" ]; then
    ZIP_ASSET="wheels-ubuntu-24.04-arm-aarch64.zip"
else
    echo "Unsupported architecture: $arch"
    exit 1
fi

# using the github api is more stable than using hardcoded links to assets
API_URL="https://api.github.com/repos/${REPO}/releases/tags/${RELEASE_TAG}"
RESPONSE=$(curl -s "$API_URL")
ZIP_ASSET_URL=$(echo "$RESPONSE" | grep "browser_download_url" | grep "$ZIP_ASSET" | sed -E 's/.*"([^"]+)".*/\1/')

if [ -z "$ZIP_ASSET_URL" ]; then
    echo "Could not find the required asset from ompl release $RELEASE_TAG"
    exit 1
fi

ZIP_FILENAME=$(basename "$ZIP_ASSET_URL")
ZIP_PATH="${TMP_DIR}/${ZIP_FILENAME}"
curl -L -o "$ZIP_PATH" "$ZIP_ASSET_URL"

unzip -q "$ZIP_PATH" -d "$TMP_DIR"
WHL_FILE=$(find "$TMP_DIR" -type f -name "*.whl" | grep "$PYTHON_VERSION")

if [ -z "$WHL_FILE" ]; then
    echo "Could not find a matching .whl file inside the zip archive."
    exit 1
fi

pip3 install "$WHL_FILE"

rm -f "$ZIP_PATH"
rm -f "$TMP_DIR/*.whl"

if ! pip3 show ompl >/dev/null 2>&1; then
    echo "OMPL installation failed."
    exit 1
else
    echo "OMPL successfully installed."
fi
