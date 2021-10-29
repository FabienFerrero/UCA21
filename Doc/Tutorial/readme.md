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
   //}
return logMsgs;
```
