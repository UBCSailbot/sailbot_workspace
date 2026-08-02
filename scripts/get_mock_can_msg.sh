#!/bin/bash
set -e

# README under sailbot_workspace/scripts/

# Pulls combined_can_frames.csv for a given OWT session from the OWT-data
# repository (https://github.com/UBCSailbot/OWT-data) and stores it at
# src/network_systems/lib/can_log/combined_can_frames.csv, where
# globals.yaml / on_water_globals.yaml point can_replay_file by default.

REPO="UBCSailbot/OWT-data"
BRANCH="main"
DEFAULT_OWT_DATE="2026-06-06"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="${SCRIPT_DIR}/../src/network_systems/lib/can_log"
DEST_FILE="combined_can_frames.csv"

# Display a help message for using this script
function helpMessage() {
    echo -e "Pull combined_can_frames.csv from the OWT-data repository in the UBCSailbot organization."
    echo -e "Usage: ./get_mock_can_msg.sh [OPTION] ..."
    echo -e "Example: ./get_mock_can_msg.sh -d 2026-06-06\n"
    echo -e "Options (All Optional):"
    echo -e "\t-d <OWT_DATE>: OWT session date to pull from, formatted yyyy-mm-dd. Defaults to ${DEFAULT_OWT_DATE}."
    echo -e "\t-h: Display this message."
}

OWT_DATE="${DEFAULT_OWT_DATE}"

# Parse command-line options (all are optional arguments)
while getopts "hd:" flag; do
    case ${flag} in
        d ) OWT_DATE="${OPTARG}" ;;
        h ) helpMessage; exit 0 ;;
        \? ) echo "Invalid option: -${flag}"; helpMessage; exit 1 ;;
        : ) echo "Option -${flag} requires an argument"; helpMessage; exit 1 ;;
    esac
done

SOURCE_URL="https://raw.githubusercontent.com/${REPO}/${BRANCH}/OWT-${OWT_DATE}/can_messages/combined_can_frames.csv"

mkdir -p "${DEST_DIR}"

echo "Pulling combined_can_frames.csv for OWT-${OWT_DATE}..."
echo "${SOURCE_URL}"

if ! curl -f -L -o "${DEST_DIR}/${DEST_FILE}" "${SOURCE_URL}"; then
    echo "Failed to download ${SOURCE_URL}"
    echo "Check that OWT-${OWT_DATE} exists and has a can_messages/combined_can_frames.csv file: https://github.com/${REPO}/tree/${BRANCH}"
    exit 1
fi

echo "Saved to ${DEST_DIR}/${DEST_FILE}"
