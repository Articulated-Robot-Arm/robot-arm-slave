#!/bin/bash

# MicroController-Deployer

ip_list=()
while IFS= read -r ip; do
    ip_list+=("$ip")
done < <(sudo arp-scan --interface=enp36s0 --localnet | awk '/^[0-9]/ && !/Routerboard\.com/ {print $1}' | grep -E '^([0-9]{1,3}\.){3}[0-9]{1,3}$')
# sudo arp-scan --interface=enp36s0 --localnet | awk '/^[0-9]/ && !/Routerboard\.com/ {print $1}' | grep -E '^([0-9]{1,3}\.){3}[0-9]{1,3}$'

# Print the list to verify
for ip in "${ip_list[@]}"; do
    echo "$ip"
    # ssh into mcs
    # verify docker is installed
    # pull newest docker container from registry
    # run container
done
