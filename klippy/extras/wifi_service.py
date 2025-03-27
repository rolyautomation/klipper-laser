# Support wifi service 
#
# Copyright (C) 2024-2028  jinqiang <jinqiang@ecomedge.io>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import subprocess
import re
import os
import time
import logging
#import json

SIGNAL_STRENGTH_MIN = -100
#SIGNAL_STRENGTH_MIN = -80
#SSID_MIN_LEN = -1
SSID_MIN_LEN = 0

class WifiService:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.interface = "wlan0"
        self.wpa_supplicant_conf = "/etc/wpa_supplicant/wpa_supplicant.conf"
        self.wired_interface = "eth0"

        self.wireless_connection = None

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('NET_SCAN', self.cmd_NET_SCAN,
                               desc=self.cmd_NET_SCAN_help)            

        gcode.register_command('NET_SCAN2', self.cmd_NET_SCAN2,
                               desc=self.cmd_NET_SCAN2_help)            

        gcode.register_command('NET_STATUS', self.cmd_NET_STATUS,
                               desc=self.cmd_NET_STATUS_help) 

        gcode.register_command('NET_CONNECT', self.cmd_NET_CONNECT,
                               desc=self.cmd_NET_CONNECT_help)   

        gcode.register_command('NET_LIST', self.cmd_NET_LIST,
                               desc=self.cmd_NET_LIST_help)                                 

        gcode.register_command('NET_FORGET', self.cmd_NET_FORGET,
                               desc=self.cmd_NET_FORGET_help)  


    cmd_NET_SCAN_help = "scan wifi network"
    def cmd_NET_SCAN(self, gcmd): 
        networks = self.scan_networks()
        networks_display = self.display_networks_ssid(networks)
        msg = "scan wifi network:"
        m = "%s%s" % (msg, networks_display)
        gcmd.respond_info(m)  

    cmd_NET_SCAN2_help = "scan wifi network"
    def cmd_NET_SCAN2(self, gcmd): 
        networks = self.scan_networks()
        msg = "scan wifi network:"
        m = "%s%s" % (msg, networks)
        gcmd.respond_info(m)  
    
    cmd_NET_STATUS_help = "get wifi status"
    def cmd_NET_STATUS(self, gcmd): 
        #status = self.get_current_connection()
        status = self.get_connection()
        msg = "get wifi status:"
        m = "%s%s" % (msg,status)
        gcmd.respond_info(m)  

    cmd_NET_CONNECT_help = "connect wifi network"
    def cmd_NET_CONNECT(self, gcmd): 
        ssid = gcmd.get('SSID', None)
        password = gcmd.get('PASSWORD', None)
        if ssid is None:
            gcmd.respond_info("SSID is required")
            return
        if password is None:
            self.connect(ssid)
        else:
            self.connect(ssid, password)
        msg = "connect wifi status:"
        m = "%s%s" % (msg, self.get_current_connection())
        gcmd.respond_info(m)  

    cmd_NET_FORGET_help = "forget wifi network"
    def cmd_NET_FORGET(self, gcmd): 
        ssid = gcmd.get('SSID', None)
        if ssid is None:
            gcmd.respond_info("SSID is required")
            return

        networks = self.list_saved_networks()
        network_found = False

        for network in networks:
            if network['ssid'] == ssid:
                self.forget_network(network['id'])
                network_found = True
                break

        if not network_found:
            m = "Network not found,ssid:%s" % (ssid,)
            gcmd.respond_info(m)
            return
        msg = "forget wifi status:"
        m = "ssid:%s wifi status:%s" % (ssid, self.get_current_connection())
        gcmd.respond_info(m)  


    cmd_NET_LIST_help = "list wifi network"
    def cmd_NET_LIST(self, gcmd): 
        saved_networks = self.list_saved_networks()
        msg = "list saved wifi network:"
        m = "%s%s" % (msg, saved_networks)
        gcmd.respond_info(m)  


    def _run_command(self, command, need_root=False):

        """Run shell command and return output"""
        if need_root and os.geteuid() != 0:
            command = ["sudo"] + command
            
        logging.info(f"Running command: {' '.join(command)}")
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        stdout, stderr = process.communicate()
        if process.returncode != 0:
            logging.error(f"Command failed: {stderr}")
        return stdout, stderr, process.returncode   

    def is_interface_up(self, interface):
        """Check if network interface is up"""
        stdout, _, _ = self._run_command(["cat", f"/sys/class/net/{interface}/operstate"])
        # if interface is up, output should be "up"
        return stdout.strip().lower() == "up"

    def scan_networks(self):

        # Ensure interface is enabled
        if not self.is_interface_up(self.interface):
            self._run_command(["ifconfig", self.interface, "up"], need_root=True)


        stdout, stderr, return_code = self._run_command(
            ["iwlist", self.interface, "scan"], 
            need_root=True
        )
        
        if return_code != 0:
            logging.error(f"Scan failed: {stderr}")
            return []

        # Parse scan results
        networks = []
        current_cell = {}
        cell_pattern = re.compile(r'Cell \d+ - ')
        ssid_pattern = re.compile(r'ESSID:"([^"]*)"')
        quality_pattern = re.compile(r'Quality=(\d+)/(\d+)')
        signal_pattern = re.compile(r'Signal level=(-\d+) dBm')
        encryption_pattern = re.compile(r'Encryption key:(\w+)')
        
        for line in stdout.split('\n'):
            line = line.strip()
            
            # New network cell
            if cell_pattern.match(line):
                if current_cell and 'ssid' in current_cell:
                    if current_cell['signal'] > SIGNAL_STRENGTH_MIN and len(current_cell['ssid']) > SSID_MIN_LEN:
                        networks.append(current_cell)
                current_cell = {}
                continue
            # Extract SSID
            ssid_match = ssid_pattern.search(line)
            if ssid_match:
                current_cell['ssid'] = ssid_match.group(1)
                current_ssid_connected = self.is_current_connection_ssid(current_cell['ssid'])
                current_cell['connected'] = current_ssid_connected
            
            # Extract signal quality
            quality_match = quality_pattern.search(line)
            if quality_match:
                quality = int(quality_match.group(1)) / int(quality_match.group(2)) * 100
                current_cell['quality'] = int(quality)
            
            # Extract signal strength
            signal_match = signal_pattern.search(line)
            if signal_match:
                current_cell['signal'] = int(signal_match.group(1))
            
            # Extract encryption status
            encryption_match = encryption_pattern.search(line)
            if encryption_match:
                current_cell['encrypted'] = encryption_match.group(1) == "on"
        
        # Add last network
        if current_cell and 'ssid' in current_cell:
            if current_cell['signal'] > SIGNAL_STRENGTH_MIN and len(current_cell['ssid']) > SSID_MIN_LEN:
                networks.append(current_cell)
        
        # Sort by signal strength
        networks.sort(key=lambda x: x.get('quality', 0), reverse=True)
        
        return networks

    def display_networks_ssid(self, networks):
        simplified_networks = []
        for network in networks:
            simplified_network = {
                'ssid': network.get('ssid', ''),
                'connected': network.get('connected', False)
            }
            simplified_networks.append(simplified_network)
        return simplified_networks



    def is_current_connection_ssid(self, ssid):
        if self.wireless_connection is None:
            return False
        if  self.wireless_connection['wireless_connected'] is False:
            return False
        if len(ssid) == 0:
            return False    
        return self.wireless_connection['ssid'] == ssid    
        
    def get_current_connection(self):
        """Get current connection information"""
        stdout, stderr, return_code = self._run_command(["iwconfig", self.interface])
        
        if return_code != 0:
            logging.error(f"Failed to get current connection: {stderr}")
            return None
        
        # Extract SSID
        ssid_match = re.search(r'ESSID:"([^"]*)"', stdout)
        if not ssid_match or ssid_match.group(1) == "":
            return None
        

        connection = {
            'ssid': ssid_match.group(1),
            'wireless_connected': True
        }
        
        # Extract signal strength
        signal_match = re.search(r'Signal level=(-\d+) dBm', stdout)
        if signal_match:
            connection['wireless_signal'] = int(signal_match.group(1))
        
        # Get IP address
        ip_stdout, _, _ = self._run_command(["ip", "addr", "show", self.interface])
        ip_match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', ip_stdout)
        if ip_match:
            connection['wireless_ip_address'] = ip_match.group(1)
        
        return connection


    def get_wired_connection(self, interface="eth0"):
        """Get wired connection information"""
        # Check if interface exists and is up
        stdout, stderr, return_code = self._run_command(["ip", "link", "show", interface])
        
        if return_code != 0:
            logging.error(f"Failed to get wired connection status: {stderr}")
            return None
            
        # Check if interface is up
        is_up = "state UP" in stdout
        
        if not is_up:
            return {
                'wired_connected': False,
                'wired_interface': interface
            }
        
        # Get IP address
        ip_stdout, _, ip_return_code = self._run_command(["ip", "addr", "show", interface])
        
        connection = {
            'wired_connected': True,
            'wired_interface': interface
        }
        
        # Extract IP address
        ip_match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', ip_stdout)
        if ip_match:
            connection['wired_ip_address'] = ip_match.group(1)
        
        # Get link speed and other information
        ethtool_stdout, _, _ = self._run_command(["ethtool", interface])
        
        # Extract link speed
        speed_match = re.search(r'Speed: (\d+[GMK]b/s)', ethtool_stdout)
        if speed_match:
            connection['wired_speed'] = speed_match.group(1)
            
        # Extract duplex mode
        duplex_match = re.search(r'Duplex: (\w+)', ethtool_stdout)
        if duplex_match:
            connection['wired_duplex'] = duplex_match.group(1)
        
        return connection   
    

    def get_connection(self):
        """Get current connection information"""
        wireless_conn = self.get_current_connection()
        self.wireless_connection = wireless_conn
        wired_conn = self.get_wired_connection(self.wired_interface)
        return {
            'wireless': wireless_conn if wireless_conn is not None else {'wireless_connected': False},
            'wired': wired_conn if wired_conn is not None else {'wired_connected': False}
        }
        

    def connect(self, ssid, password=None):
        """Connect to specified wireless network"""
        # Create wpa_cli command
        if password:
            add_cmd = ["wpa_cli", "-i", self.interface, "add_network"]
            stdout, _, _ = self._run_command(add_cmd)
            network_id = stdout.strip()

            self._run_command(["wpa_cli", "-i", self.interface, "set_network", network_id, "ssid", f'"{ssid}"'])
            self._run_command(["wpa_cli", "-i", self.interface, "set_network", network_id, "psk", f'"{password}"'])
            self._run_command(["wpa_cli", "-i", self.interface, "enable_network", network_id])
            self._run_command(["wpa_cli", "-i", self.interface, "save_config"])
        else:
            # Open network
            add_cmd = ["wpa_cli", "-i", self.interface, "add_network"]
            stdout, _, _ = self._run_command(add_cmd)
            network_id = stdout.strip()
            
            self._run_command(["wpa_cli", "-i", self.interface, "set_network", network_id, "ssid", f'"{ssid}"'])
            self._run_command(["wpa_cli", "-i", self.interface, "set_network", network_id, "key_mgmt", "NONE"])
            self._run_command(["wpa_cli", "-i", self.interface, "enable_network", network_id])
            self._run_command(["wpa_cli", "-i", self.interface, "save_config"])

        # reconnect
        self._run_command(["wpa_cli", "-i", self.interface, "reconfigure"])
        
        # Wait for connection
        for _ in range(30):  # Wait up to 30 seconds
            time.sleep(1)
            connection = self.get_current_connection()
            if connection and connection['ssid'] == ssid:
                return True

        return False
    

    def list_saved_networks(self):
        # list saved networks
        stdout, _, _ = self._run_command(["wpa_cli", "-i", self.interface, "list_networks"])
       
        networks = []
        lines = stdout.strip().split('\n')
        
        # skip header
        for line in lines[1:]:
            parts = line.split('\t')
            if len(parts) >= 2:
                networks.append({
                    'id': parts[0],
                    'ssid': parts[1],
                    'flags': parts[3] if len(parts) > 3 else ""
                })
        return networks
    
    def forget_network(self, network_id):
        # delete saved network
        self._run_command(["wpa_cli", "-i", self.interface, "remove_network", str(network_id)])
        self._run_command(["wpa_cli", "-i", self.interface, "save_config"])
        return True


def load_config(config):
    return WifiService(config)
