# Cloggo

`cloggo` is a raspberry pi pico microcontroller project for determining if a plumbing filter stack eg. [several "big blue" commodity filters](https://www.pentair.com/en-us/products/business-industry/residential-water-treatment/filters-housings/big_blue_heavy_duty_series.html) are clogged and need replacement. It is implemented in Rust using [`embassy-rs`](https://github.com/embassy-rs/embassy) and the webserver is using [`picoserve`](https://crates.io/crates/picoserve).

The rough idea is as follows:

1) Put a sensor in front of and behind each filter or groups of filters you want to monitor
2) Drop a few lines of ethernet to that location
3) Calibrate the sensor by using the physical gauge to develop a linear function to translate voltage into pressure
4) Pipe the data into home assistant (measure the pressure of the sensor by doing an ADC read)
5) Take the sensors and compute the delta between them (and also use potentially more advanced heuristics) to determine if the filters are clogged
6) Use the fact that you now have pressure data to help in other useful automations, like not running the washing machine when the sprinklers are on (pressure is lower than a certain value), stuff like that.

## Why?

I live in an area where I do not have city water at the road, so my house is on a private well. Where I live, Well water has very high amounts of sediment which can clog filters. I have a sediment filter with an automatically flushing valve, but w/o hard data it is hard to know which filters in the system to replace and when. Because the plumbing in my house is located in the crawlspace, it is frustrating to have to climb down there and check on the status of filters, so hence this little automation project.

The `w5500-evb-pico` was chosen for a few reasons:

1) good software support in `embassy-rs` for the NIC
2) onboard ethernet means not having to deal with WiFi (important in a crawlspace deployment where phyiscal access to the device is limited)
3) I can power the device via a PoE splitter (there are limited outlets in most crawlspaces)

## Home Assistant Integration

`cloggo` is currently integrated with homeassistant via the Iot Polling integration, here is the example `configuration.yaml` entry:

```yaml
rest:
  - scan_interval: 20
    resource: http://192.168.1.240/status
    sensor:
      - name: "Cloggo CPU Temperature"
        value_template: '{{ value_json.temp }}'
        unit_of_measurement: '°C'
        json_attributes:
            - "temp"
```

## BOM

- Board (Raspberry Pi Pico w/ Ethernet): <https://docs.wiznet.io/Product/iEthernet/W5500/w5500-evb-pico>
- Pressure Transducer: <https://www.amazon.com/dp/B0BM6F4Q5J>
- Fittings:
  - Drop Ear Pex: <https://www.amazon.com/gp/product/B091NJWBFG>
  - 1/2 to 1/8 NPT reducer: <https://www.amazon.com/gp/product/B0773RWNP3>
- Optional:
  - Debug Probe: <https://www.amazon.com/dp/B0C5XNQ7FD>
  - Another Pressure Gauge (for calibration) <https://www.amazon.com/dp/B00KL71PHO?psc=1&ref=ppx_yo2ov_dt_b_product_details>

## Flashing

Once you have a [`probe-rs`](https://probe.rs/) and pi pico compatible debugger setup (incl. wiring), you can flash and debug the program by running:

`cargo run --release` in a terminal window.

## TODOs

- [ ] actually implement the ADC reads for the transducers (waiting on some parts)
- [x] implement a calibration routine that can be done over rest
- [x] implement saving calibration function in non-volatile memory
- [ ] MQTT integration?
- [ ] explore if multiple sensors can be managed by a single Pico
