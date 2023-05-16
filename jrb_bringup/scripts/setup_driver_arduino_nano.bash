git clone https://github.com/juliagoda/CH341SER
cd CH341SER
make
sudo make load
sudo apt autoremove brltty
sudo usermod -a -G dialout $USER