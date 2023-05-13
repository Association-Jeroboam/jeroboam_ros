#!/bin/bash

# Exit on error
# set -e

# Write a small title of the current action (first param)
function title {
		echo -e "\e[1;36m===== $1 =====\e[0m"
}

title "Remove packages"
sudo apt remove -y --autoremove snapd unattended-upgrades plymouth avahi-daemon
echo ""

title "Disable services"
sudo systemctl disable cloud-config.service
sudo systemctl disable cloud-final.service
sudo systemctl disable cloud-init
sudo systemctl disable cloud-init-local.service

sudo systemctl disable snapd.service
sudo systemctl disable snapd.seeded.service
sudo systemctl disable snapd.socket

sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service

sudo systemctl disable snapd
sudo systemctl mask snapd

sudo systemctl disable avahi-daemon
sudo systemctl mask avahi-daemon

sudo systemctl disable bluetooth.service
sudo systemctl mask bluetooth.service

sudo systemctl disable ModemManager.service
sudo systemctl mask ModemManager.service

sudo systemctl disable NetworkManager-wait-online.service
sudo systemctl mask NetworkManager-wait-online.service

sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer
echo ""

title "Disable motd news"
sudo sed -i 's/^ENABLED=.*/ENABLED=0/' /etc/default/motd-news
echo ""

title "Disable GNOME notifications"
sudo dconf write /org/gnome/desktop/notifications/show-banners false
echo ""

title "Finished"
