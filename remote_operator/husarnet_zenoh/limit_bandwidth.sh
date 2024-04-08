#!/bin/bash

# Examples 
# ./limit_bandwidth.sh
# ./limit_bandwidth.sh -i eth0

# Define the rate limiting values
RATE="100mbit" # max rate at which data can be sent
LATENCY="25ms" # time it takes packet to travel from source to destination (smaller value = more responsive but drops more packets)
BURST="10k" # number of bytest that can exceed rate limit for brief time
DEFAULT_INTERFACE=""

# Parse options
while getopts "i:" opt; do
  case $opt in
    i)
      DEFAULT_INTERFACE=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# If no interface was provided, detect the default network interface
if [ -z "$DEFAULT_INTERFACE" ]; then
  echo "WARNING: detecting network interface automatically"
  DEFAULT_INTERFACE=$(ip route | grep default | awk '{print $5}' | head -n 1)
  echo "Detected $DEFAULT_INTERFACE as default network interface"
fi

# Check if the default interface has been identified/set
if [ -z "$DEFAULT_INTERFACE" ]; then
    echo "No default network interface could be identified. Exiting."
    exit 1
fi

# Delete existing qdiscs if they exist, if they don't exist just continue
tc qdisc del dev "$DEFAULT_INTERFACE" ingress || true
tc qdisc del dev "$DEFAULT_INTERFACE" root || true

# Add new qdisc and filter rules
tc qdisc add dev "$DEFAULT_INTERFACE" handle ffff: ingress
tc filter add dev "$DEFAULT_INTERFACE" parent ffff: protocol ip prio 50 u32 match ip src 0.0.0.0/0 police rate "$RATE" burst "$BURST" drop flowid :ffff
tc qdisc add dev "$DEFAULT_INTERFACE" root tbf rate "$RATE" latency "$LATENCY" burst "$BURST"

# Show the current qdisc configuration for $DEFAULT_INTERFACE
tc qdisc show dev "$DEFAULT_INTERFACE"
