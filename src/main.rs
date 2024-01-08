//! This example implements a TCP client that attempts to connect to a host on port 1234 and send it some data once per second.
//!
//! Example written for the [`WIZnet W5500-EVB-Pico`](https://www.wiznet.io/product-item/w5500-evb-pico/) board.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::{debug, error, info, unwrap};
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::chip::W5500;
use embassy_net_wiznet::State as WizState;
use embassy_net_wiznet::*;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::clocks::RoscRng;

use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{FLASH, PIN_17, PIN_20, PIN_21, SPI0};
use embassy_rp::spi::{Async as AsyncSPI, Config as SpiConfig, Spi};
use embassy_sync::channel;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_rp::flash::{Async, ERASE_SIZE, FLASH_BASE};
use embassy_rp::flash::{Flash, Mode};
use minicbor::decode::info;
use minicbor::{Decode, Decoder, Encode, Encoder};

use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    utils::rng_generator::CountingRng,
};

use serde::{Deserialize, Serialize};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;

        ADC_IRQ_FIFO => InterruptHandler;

});

use rand::RngCore;

use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

type RunnerType = Runner<
    'static,
    W5500,
    ExclusiveDevice<Spi<'static, SPI0, AsyncSPI>, Output<'static, PIN_17>, Delay>,
    Input<'static, PIN_21>,
    Output<'static, PIN_20>,
>;

#[embassy_executor::task]
async fn ethernet_task(runner: RunnerType) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device<'static>>) -> ! {
    stack.run().await
}

const ADDR_OFFSET: u32 = 0x100000;
const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[derive(Clone, Copy)]
struct SharedADC(&'static Mutex<CriticalSectionRawMutex, Adc<'static, embassy_rp::adc::Async>>);

#[derive(Clone, Copy)]
struct SharedFlash(
    &'static Mutex<
        CriticalSectionRawMutex,
        Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
    >,
);

#[derive(Clone, serde::Serialize)]
struct Status {
    temp: f32,
}

#[derive(Clone, serde::Serialize)]
struct PressureResponse {
    pressure: f32,
    raw_reading: u16,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<PressureResponseError>,
}

#[derive(Clone, serde::Serialize)]
struct PressureResponseError {
    error: &'static str,
}

type SharedSensor = &'static Mutex<CriticalSectionRawMutex, Channel<'static>>;

#[derive(Clone, Copy)]
struct SharedPressureSensor(SharedSensor);

#[derive(Clone, Copy, Serialize)]
struct MqttMessage {
    voltage: u16,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 50_000_000;
    let (miso, mosi, clk) = (p.PIN_16, p.PIN_19, p.PIN_18);
    let spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, spi_cfg);
    let cs = Output::new(p.PIN_17, Level::High);
    let w5500_int = Input::new(p.PIN_21, Pull::Up);
    let w5500_reset = Output::new(p.PIN_20, Level::High);

    // add some delay to give an attached debug probe time to parse the
    // defmt RTT header. Reading that header might touch flash memory, which
    // interferes with flash write operations.
    // https://github.com/knurling-rs/defmt/pull/683
    Timer::after_millis(10).await;

    // let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH3);

    let mac_addr = [0x02, 0x11, 0x10, 0x10, 0x10, 0x10];

    let state = make_static!(WizState::<8, 8>::new());

    let (device, runner) = embassy_net_wiznet::new(
        mac_addr,
        state,
        ExclusiveDevice::new(spi, cs, Delay),
        w5500_int,
        w5500_reset,
    )
    .await;
    unwrap!(spawner.spawn(ethernet_task(runner)));

    let adc = Adc::new(p.ADC, Irqs, Config::default());

    let ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    // p26 is the adc input for the pressure sensor
    // p27 and p28 are not being used currently
    let p26 = Channel::new_pin(p.PIN_26, Pull::None);

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    let stack = &*make_static!(Stack::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        make_static!(StackResources::<6>::new()),
        seed
    ));

    // Launch network task
    unwrap!(spawner.spawn(net_task(stack)));

    info!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    info!("IP address: {:?}", local_addr);

    let shared_adc = SharedADC(make_static!(Mutex::new(adc)));
    let shared_pressure = SharedPressureSensor(make_static!(Mutex::new(p26)));

    // run the listen task
    unwrap!(spawner.spawn(mqtt_publisher(stack, shared_adc, shared_pressure)));
}

#[embassy_executor::task(pool_size = 1)]
async fn mqtt_publisher(
    stack: &'static Stack<Device<'static>>,
    shared_adc: SharedADC,
    shared_pressure: SharedPressureSensor,
) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    let mut config: ClientConfig<'_, 10, CountingRng> = ClientConfig::new(
        rust_mqtt::client::client_config::MqttVersion::MQTTv5,
        CountingRng(20000),
    );
    config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
    config.add_client_id("cloggo");

    config.max_packet_size = 100;
    let mut recv_buffer = [0; 80];
    // writer buffer needs to be large enough to hold the auto discovery message
    let mut write_buffer = [0; 600];

    let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    let host_addr = embassy_net::IpAddress::v4(192, 168, 1, 9);
    socket
        .connect((host_addr, 1883))
        .await
        .expect("Failed to open socket connection");

    let mut client = MqttClient::new(socket, &mut write_buffer, 600, &mut recv_buffer, 80, config);
    client
        .connect_to_broker()
        .await
        .expect("Failed to connect to broker");

    // send the auto discovery message (raw json)
    let auto_discovery_message = r#"{
        "name": "PostFilterPressureSensor",
        "device_class": "voltage",
        "state_topic": "homeassistant/sensor/cloggo01/state",
        "unit_of_measurement": "mv",
        "value_template": "{{ value_json.voltage }}",
        "unique_id": "cloggo01",
        "device": {
            "identifiers": ["cloggo01"],
            "name": "Post Filter Pressure Sensor",
            "sw_version": "0.1",
            "model": "Custom Pressure Sensor",
            "manufacturer": "Will Eaton <me@wseaton.com>"
        }
    }"#;

    let message_size = auto_discovery_message.as_bytes().len();
    info!("Sending auto discovery message of size {}", message_size);

    match client
        .send_message(
            "homeassistant/sensor/cloggo/config",
            auto_discovery_message.as_bytes(),
            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
            true,
        )
        .await
    {
        Ok(_) => {
            info!("Auto discovery message sent");
        }
        Err(e) => {
            error!("Error sending auto discovery message: {:?}", e);
        }
    };

    loop {
        // do an ADC read
        let res = {
            let mut temp_channel = shared_pressure.0.lock().await;
            shared_adc
                .0
                .lock()
                .await
                .read(&mut temp_channel)
                .await
                .unwrap_or(0)
        };

        let message = MqttMessage { voltage: res };

        let mut message_buffer: [u8; 200] = [0; 200];

        let serialized_data = serde_json_core::to_slice(&message, &mut message_buffer).unwrap();
        let used_buffer = &message_buffer[..serialized_data]; // Slicing the buffer to the used size

        match client
            .send_message(
                "homeassistant/sensor/cloggo01/state",
                used_buffer,
                rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                true,
            )
            .await
        {
            Ok(_) => {
                info!("Message sent");
            }
            Err(e) => {
                error!("Error sending message: {:?}", e);
            }
        };

        // sleep
        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn wait_for_config(stack: &'static Stack<Device<'static>>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}
