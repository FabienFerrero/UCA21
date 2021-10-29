JSON funtion to decode TTN V3 MQTT packet in Node-red :

```
var logMsgs = [];
logMsgs[0]=({payload: {
    rssi: msg.payload.uplink_message.rx_metadata[0].rssi,
    voltage: msg.payload.uplink_message.decoded_payload.analog_in_3,
    temperature:msg.payload.uplink_message.decoded_payload.temperature_1,
    Humidity: msg.payload.uplink_message.decoded_payload.relative_humidity_2, 
    luminosity: msg.payload.uplink_message.decoded_payload.luminosity_4
                            }
                 });
return logMsgs;
```



JSON funtion to extract a value TTN V3 MQTT packet in Node-red and plot it in dashboard :
```
var tmp = {};
tmp.payload = msg.payload.uplink_message.decoded_payload.luminosity_4;
return tmp;
```

JSON funtion to extract a value TTN V3 MQTT packet in Node-red and send an email:
```
var tmp = {};
var lum = msg.payload.uplink_message.decoded_payload.luminosity_4;
tmp.topic = "information capteur";
if (lum>200)
tmp.payload = "il y a de la lumiere";
else
tmp.payload = "il n'y a pas de lumiere";
return tmp;
```
