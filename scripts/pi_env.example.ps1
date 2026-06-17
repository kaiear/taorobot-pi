# Copy this file to scripts/pi_env.ps1 and edit values for your Raspberry Pi.

$PI_HOST_TAILSCALE = "100.124.15.24"
$PI_HOST_LAN = "192.168.137.100"
$PI_HOST = $PI_HOST_TAILSCALE
$PI_USER = "ubuntu"
$PI_KEY = "C:/Users/kaier/.ssh/id_rsa_pi4b"
$PI_WS = "/home/ubuntu/tao_robot_ws"
$PI_SRC = "$PI_WS/src"
