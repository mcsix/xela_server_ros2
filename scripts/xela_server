#!/bin/bash

params=("$@")

filtered_params=()

printf '%s ' "${params[@]}"

# Find and remove '--ros-args' and '--params-file' from the list
while [[ " ${params[*]} " == *" --ros-args "* || " ${params[*]} " == *" --params-file "* ]]; do
    index=$(( ${#params[@]} - 1 ))
    for (( i=0; i<${#params[@]}; i++ )); do
        if [ "${params[$i]}" == "--ros-args" ]; then
            index=$i
            printf "\nFound '--ros-args' at index ${index}"
            break
        elif [ "${params[$i]}" == "--params-file" ]; then
            index=$i
            printf "\nFound '--params-file' at index ${index}"
            break
        fi
    done
    params=("${params[@]:0:$index}" "${params[@]:$((index+2))}")

    # Check if "--ros-args" includes "__node:=xela_server" and remove it
    if [ "${params[$index]}" == "__node:=xela_server" ]; then
        printf "\nRemoving __node at index ${index}"
        params=("${params[@]:0:$index}" "${params[@]:$((index+1))}")
    fi
done
printf "\n"

# Build the filtered parameters
filtered_params=("${params[@]}")

# Print the command (for debugging purposes)
echo "Running command: xela_server ${params[@]}"

# Execute the command
xela_server "${filtered_params[@]}"