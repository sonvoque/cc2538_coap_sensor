cc2538_coap_sensor
==================

WSN temperature sensor based on cc2538 + Contiki + CoAP

git clone https://github.com/retfie/cc2538_coap_sensor.git
cd cc2538_coap_sensor
git submodule init
git submodule update
make

cd contiki
git submodule update --init


cd ../
# Put device in bootloader mode via external pin
make coap-post.upload

If flashing was successfull, sensor will print its IPv6 address on console
and will start posting messages with some metrics to predefined IP address bbbb::1

# Messages are in JSON format
{ "eui": "00124b0003d0a6ac", "vdd": "2671 mV", "temp": "35238 mC", "count": "5242", "tmp102": "25312 mC", "rssi": "-105 dBm" }

You can change some of parameters with CoAP capable client:
- Copper (Cu) plugin for Firefox
- SMCP C stack from command line ( https://github.com/darconeous/smcp.git )

# Get current value of some parameter
smcpctl get coap://[IPv6 addr of sensor]/config?param=interval

# Change parameter on device
smcpctl post coap://[IPv6 addr of sensor]/config?param=interval 60

# Available parameters currently are:
interval - time between metric post
path	 - path to which to post on server
ip	 - IP of the CoAP server to post


