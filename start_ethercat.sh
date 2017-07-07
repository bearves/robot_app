#!/bin/bash

sudo /opt/etherlab/etc/init.d/ethercat restart
sleep 4
sudo /opt/etherlab/bin/ethercat sla

