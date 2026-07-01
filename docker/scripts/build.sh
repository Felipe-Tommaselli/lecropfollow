#!/bin/bash
# Use Bash as the shell for this script.

# Check if the current directory when running the command is correct.
if [[ $PWD = *lecropfollow ]]; then
    # If the current directory ends with 'lecropfollow', build from here with docker as context
    docker build \
        --network=host \
        -f docker/ROS_noetic.dockerfile \
        -t tommaselli/lecropfollow:noetic \
        --rm \
        .
elif [[ $PWD = *lecropfollow/docker ]]; then
    # If in docker directory, go back to parent and build
    cd ..
    docker build \
        --network=host \
        -f docker/ROS_noetic.dockerfile \
        -t tommaselli/lecropfollow:noetic \
        --rm \
        .
else
    # If the current directory is not correct, print an error message.
    echo -e "You must be in either 'lecropfollow' or the 'lecropfollow/docker' directory to run this command."
    # Exit the script with status 1 to indicate an error.
    return 1
fi