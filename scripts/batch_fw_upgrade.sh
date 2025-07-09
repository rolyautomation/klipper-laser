#!/bin/bash

# Batch Firmware Upgrade Script
# This script calls fw_upgrade.sh to perform batch firmware upgrades

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Check if running with root privileges
if [ "$EUID" -ne 0 ]; then
    log_error "Please run this script with sudo"
    exit 1
fi

# Set firmware directory
FIRMWARE_DIR="."
if [ $# -eq 1 ]; then
    FIRMWARE_DIR="$1"
fi

# Check if firmware directory exists
if [ ! -d "$FIRMWARE_DIR" ]; then
    log_error "Firmware directory $FIRMWARE_DIR does not exist"
    exit 1
fi

# Check if fw_upgrade.sh script exists
UPGRADE_SCRIPT="$(dirname "$0")/fw_upgrade.sh"
if [ ! -f "$UPGRADE_SCRIPT" ]; then
    log_error "Upgrade script $UPGRADE_SCRIPT does not exist"
    exit 1
fi

# Ensure script has execution permissions only if it doesn't already have them
if [ ! -x "$UPGRADE_SCRIPT" ]; then
    log_info "Adding execution permission to $UPGRADE_SCRIPT"
    chmod +x "$UPGRADE_SCRIPT"
fi

# Display menu
show_menu() {
    clear
    echo "=========================================="
    echo "       Batch Firmware Upgrade Tool"
    echo "=========================================="
    echo "1. Scan and upgrade all devices"
    echo "2. Upgrade specific device"
    echo "3. View connected devices"
    echo "4. View available firmware"
    echo "0. Exit"
    echo "=========================================="
    read -p "Please select an option [0-4]: " choice
    
    case $choice in
        1) upgrade_all_devices ;;
        2) upgrade_specific_device ;;
        3) list_devices ;;
        4) list_firmware ;;
        0) exit 0 ;;
        *) log_error "Invalid selection"; sleep 2; show_menu ;;
    esac
}

# List all connected devices
list_devices() {
    log_info "Scanning connected devices..."
    
    if [ ! -d "/dev/serial/by-id" ]; then
        log_error "Directory /dev/serial/by-id does not exist"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    DEVICES=($(ls -1 /dev/serial/by-id/ | grep -E "rp2040"))
    
    if [ ${#DEVICES[@]} -eq 0 ]; then
        log_warn "No compatible devices found"
    else
        log_info "Found ${#DEVICES[@]} device(s):"
        for i in "${!DEVICES[@]}"; do
            echo "[$((i+1))] ${DEVICES[$i]}"
        done
    fi
    
    read -p "Press any key to continue..." -n1
    show_menu
}

# List available firmware
list_firmware() {
    log_info "Scanning available firmware files..."
    
    #FIRMWARE_FILES=($(find "$FIRMWARE_DIR" -name "*.uf2" | sort))
    FIRMWARE_FILES=($(find "$FIRMWARE_DIR" -maxdepth 1 -name "*.uf2" | sort))
    
    if [ ${#FIRMWARE_FILES[@]} -eq 0 ]; then
        log_warn "No firmware files found in $FIRMWARE_DIR directory"
    else
        log_info "Found ${#FIRMWARE_FILES[@]} firmware file(s):"
        for i in "${!FIRMWARE_FILES[@]}"; do
            echo "[$((i+1))] ${FIRMWARE_FILES[$i]}"
        done
    fi
    
    read -p "Press any key to continue..." -n1
    show_menu
}

# Upgrade all devices
upgrade_all_devices() {
    log_info "Starting batch upgrade of all devices..."
    
    # Call fw_upgrade.sh script
    #"$UPGRADE_SCRIPT" "$FIRMWARE_DIR"
    log_info "to be do"
    
    log_info "Batch upgrade completed"
    read -p "Press any key to continue..." -n1
    show_menu
}

# Upgrade specific device
upgrade_specific_device() {
    log_info "Scanning connected devices..."
    
    if [ ! -d "/dev/serial/by-id" ]; then
        log_error "Directory /dev/serial/by-id does not exist"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    DEVICES=($(ls -1 /dev/serial/by-id/ | grep -E "rp2040"))
    
    if [ ${#DEVICES[@]} -eq 0 ]; then
        log_warn "No compatible devices found"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    log_info "Found ${#DEVICES[@]} device(s):"
    for i in "${!DEVICES[@]}"; do
        echo "[$((i+1))] ${DEVICES[$i]}"
    done
    
    read -p "Select device number to upgrade [1-${#DEVICES[@]}]: " device_choice
    
    if ! [[ "$device_choice" =~ ^[0-9]+$ ]] || [ "$device_choice" -lt 1 ] || [ "$device_choice" -gt ${#DEVICES[@]} ]; then
        log_error "Invalid selection"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    selected_device=${DEVICES[$((device_choice-1))]}
    
    # Extract device prefix
    prefix=$(echo "$selected_device" | sed -n 's/^usb-\([^_]*_[^_]*\)_.*/\1/p')
    log_info "Selected device prefix: $prefix"

    if [ -z "$prefix" ]; then
        log_error "Could not extract prefix from device name: $selected_device"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    # Find matching firmware file
    firmware_files=("$FIRMWARE_DIR/${prefix}"*.uf2)
    log_info "Firmware files found: ${firmware_files[@]}"
    
    if [ ${#firmware_files[@]} -eq 0 ] || [ ! -f "${firmware_files[0]}" ]; then
        log_error "No matching firmware file found for device: $selected_device (prefix: $prefix)"
        read -p "Press any key to continue..." -n1
        show_menu
        return
    fi
    
    # If multiple files match, use the most recent one
    latest_firmware=$(ls -t "${firmware_files[@]}" | head -n1)
    
    log_info "Will use firmware file: $latest_firmware"
    read -p "Confirm upgrade? (y/n): " confirm
    
    if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
        # Create temporary directory for single device upgrade
        #temp_dir=$(mktemp -d)
        #cp "$latest_firmware" "$temp_dir/"
        
        # Call fw_upgrade.sh script to upgrade single device
        "$UPGRADE_SCRIPT" "$selected_device" "$latest_firmware"
        
        # Clean up temporary directory
        #rm -rf "$temp_dir"
        
        log_info "Device upgrade completed"
    else
        log_info "Upgrade cancelled"
    fi
    
    read -p "Press any key to continue..." -n1
    show_menu
}

# Display main menu
show_menu