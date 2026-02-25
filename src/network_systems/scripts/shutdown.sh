#!/bin/bash
# FOR THE SCRIPT TO WORK:
# Since we are interacting with the host machine, there are some prerequisities for the host that can't be git tracked inside this repo.
# 1. There must be an SSH key that is mounted into the dev container, and it's corresponding public key exists in soft/.ssh/authorized_keys
# 2. The 'soft' user has sudo-less access to the shutdown command, currently configured in /etc/sudoers.d/shutdown-nopass by the line:
#       soft ALL=(ALL) NOPASSWD: /sbin/shutdown
ssh soft@localhost -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "sudo shutdown"

# Note: the flags are to avoid the confirmation message "The authenticity of host 'localhost (::1)' can't be established" on the first connection,
# because it requires a confirmation
