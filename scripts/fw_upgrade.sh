#!/bin/bash

# Auto-upgrade script
# Usage: ./upgrade.sh <device_id> <firmware_file>
# Example: ./upgrade.sh usb-mfiber_rp2040_E6646C858B91842A-if00 ../klipper_mfiber_usb_250709.uf2

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
check_root() {
    if [ "$EUID" -ne 0 ]; then
        log_error "Please run this script with sudo"
        exit 1
    fi
}

# Parameter validation
if [ $# -ne 2 ]; then
    echo "Usage: $0 <device_id> <firmware_file>"
    echo "Example: $0 usb-mfiber_rp2040_E6646C858B91842A-if00 ../klipper_mfiber_usb_250709.uf2"
    exit 1
fi

DEVICE_ID=$1
FIRMWARE_FILE=$2
DEVICE_PATH="/dev/serial/by-id/$DEVICE_ID"
MOUNT_POINT="/mnt/usb"

# Check if firmware file exists
if [ ! -f "$FIRMWARE_FILE" ]; then
    log_error "Firmware file $FIRMWARE_FILE does not exist"
    exit 1
fi

# Check if device exists
if [ ! -L "$DEVICE_PATH" ]; then
    log_error "Device $DEVICE_PATH does not exist"
    exit 1
fi

# Main function
main() {
    # Check root privileges
    check_root
    
    # Check if Klipper service is running and stop it if needed
    log_info "Checking Klipper service status..."
    if systemctl is-active --quiet klipper; then
        log_info "Stopping Klipper service..."
        service klipper stop
        if [ $? -ne 0 ]; then
            log_error "Failed to stop Klipper service"
            exit 1
        fi
    else
        log_info "Klipper service is already stopped"
    fi
    
    # Use flashtool.py to flash the device
    log_info "Preparing to flash device $DEVICE_PATH..."
    python3 flashtool.py -d "$DEVICE_PATH" -b 250000 -r
    if [ $? -ne 0 ]; then
        log_error "Failed to flash device"
        #log_info "Attempting to restart Klipper service..."
        #service klipper start
        exit 1
    fi
    
    # Wait for device re-enumeration
    log_info "Waiting for device re-enumeration..."
    sleep 3
    
    # Display block device information
    log_info "Displaying block device information:"
    lsblk
    
    # Check if mount point exists, create if not
    if [ ! -d "$MOUNT_POINT" ]; then
        log_info "Creating mount point $MOUNT_POINT..."
        mkdir -p "$MOUNT_POINT"
        if [ $? -ne 0 ]; then
            log_error "Failed to create mount point"
            exit 1
        fi
    fi
    
    # Check if USB device is available
    if [ ! -b "/dev/sda1" ]; then
        log_error "USB device /dev/sda1 not found"
        #log_info "Attempting to restart Klipper service..."
        #service klipper start
        exit 1
    fi
    
    # Mount USB device
    log_info "Mounting USB device..."
    mount /dev/sda1 "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        log_error "Failed to mount USB device"
        #log_info "Attempting to restart Klipper service..."
        #service klipper start
        exit 1
    fi
    
    # Copy firmware file
    log_info "Copying firmware file $FIRMWARE_FILE to $MOUNT_POINT..."
    cp "$FIRMWARE_FILE" "$MOUNT_POINT/"
    if [ $? -ne 0 ]; then
        log_error "Failed to copy firmware file"
        log_info "Attempting to unmount USB device..."
        umount "$MOUNT_POINT"
        #log_info "Attempting to restart Klipper service..."
        #service klipper start
        exit 1
    fi
    
    # Ensure file write completion
    #sync
    
    # Unmount USB device
    log_info "Unmounting USB device..."
    umount "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        log_error "Failed to unmount USB device"
        exit 1
    fi
    
    # Wait for device to reboot
    log_info "Waiting for device to reboot..."
    sleep 5
    
    # Restart Klipper service
    log_info "Restarting Klipper service..."
    service klipper start
    if [ $? -ne 0 ]; then
        log_error "Failed to restart Klipper service"
        exit 1
    fi
    
    log_info "Upgrade completed successfully!"
}

# Execute main function
main
