## Envorino

A small ESP8266-based project for reporting environmental data over MQTT.

### Why?

In search of an IoT-connected "weather station", or enviromental sensor, I couldn't quite find what I wanted, and instead set out to build my own, if nothing else, to see what the options were and how easily it could be done. As it turns out, it's fairly straight-forward and gives you infinitely more control than anything I could find on the market.

### Parts

This project is ESP8266-based, and in my case, I'm using a Firebeetle ESP8266, simply since it was readily available, but any variant of this chipset should do just fine. I hear the NodeMCU boards are nice.

As for the sensor, a lot of projects use the DHT11/DHT22 sensors, and that was also originally the plan, until I stumbled over the Bosch BME680's, which in addition to temperature and humidity, also include barometric pressure, altitude, and a *gas resistance* sensor. This means you can read IAQ (indoor air quality) readings as well, which suddenly seemed like a necessary measure.

Finally, these should be easy to deploy, so input power via micro-USB was a necessity for me, as was [a 3D-printed case](https://www.thingiverse.com/thing:2841583). I intend to build and deploy 5 or 6 of these and scatter them around my house.

### Receiving the data

Seeing as I wanted to display the environmental data in Grafana, I eventually ended up settling on using MQTT to transport the measurements to a central collection server. I'm already using Telegraf quite extensively to collect metrics over SNMP and a number of shellscripts, piping them to InfluxDB, so given that there's good support for MQTT, it was a natural choice. Recommend using the mosquitto server to collect/debug. You can run this on a Raspberry PI or something, or as I do, on a virtualized Ubuntu server under ESXi.

### Sensor name

Instead of having to change the sensor "name" for each of these that I deploy, it instead obtains its name from the MAC address of the board. You'll need to edit the firmware to include your board in specific, or it'll simply report as "unbound". The ESP8266 WiFi lbrary makes this very easy, and this code echo's out its own MAC address on bootup over serial, too (at 115200 baud).