#!/bin/bash
apt-get install lighttpd
easy_install web.py
cp -f ./lighttpd.conf /etc/lighttpd/
sudo cp -r ./cgi-bin/ /var/www/
