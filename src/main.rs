//! This example implements a TCP client that attempts to connect to a host on port 1234 and send it some data once per second.
//!
//! Example written for the [`WIZnet W5500-EVB-Pico`](https://www.wiznet.io/product-item/w5500-evb-pico/) board.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::chip::W5500;
use embassy_net_wiznet::State as WizState;
use embassy_net_wiznet::*;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIN_17, PIN_20, PIN_21, SPI0};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration};
use embedded_hal_bus::spi::ExclusiveDevice;

use picoserve::extract::State;
use picoserve::response::IntoResponse;
use picoserve::routing::{get, parse_path_segment};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;

        ADC_IRQ_FIFO => InterruptHandler;

});

use rand::RngCore;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn ethernet_task(
    runner: Runner<
        'static,
        W5500,
        ExclusiveDevice<Spi<'static, SPI0, Async>, Output<'static, PIN_17>, Delay>,
        Input<'static, PIN_21>,
        Output<'static, PIN_20>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device<'static>>) -> ! {
    stack.run().await
}

struct EmbassyTimer;

impl picoserve::Timer for EmbassyTimer {
    type Duration = embassy_time::Duration;
    type TimeoutError = embassy_time::TimeoutError;

    async fn run_with_timeout<F: core::future::Future>(
        &mut self,
        duration: Self::Duration,
        future: F,
    ) -> Result<F::Output, Self::TimeoutError> {
        embassy_time::with_timeout(duration, future).await
    }
}

#[derive(Clone, Copy)]
struct SharedTempSensor(&'static Mutex<CriticalSectionRawMutex, Channel<'static>>);

#[derive(Clone, Copy)]
struct SharedADC(&'static Mutex<CriticalSectionRawMutex, Adc<'static, embassy_rp::adc::Async>>);

struct AppState {
    temp_sensor: SharedTempSensor,
    adc: SharedADC,
}

type AppRouter = impl picoserve::routing::PathRouter<AppState>;

impl picoserve::extract::FromRef<AppState> for SharedTempSensor {
    fn from_ref(state: &AppState) -> Self {
        state.temp_sensor
    }
}

impl picoserve::extract::FromRef<AppState> for SharedADC {
    fn from_ref(state: &AppState) -> Self {
        state.adc
    }
}

#[derive(Clone, serde::Serialize)]
struct Status {
    temp: f32,
}

async fn get_status(
    State(SharedTempSensor(temp)): State<SharedTempSensor>,
    State(SharedADC(adc)): State<SharedADC>,
) -> impl IntoResponse {
    let res = {
        let mut temp_channel = temp.lock().await;
        adc.lock().await.read(&mut temp_channel).await.unwrap_or(0)
    }; // Locks are dropped here because of the block scope.
       // Now process 'res' without holding any locks.
    picoserve::response::Json(Status {
        temp: convert_to_celsius(res),
    })
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

    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00];

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

    // let mut p26 = Channel::new_pin(p.PIN_26, Pull::None);
    // let mut p27 = Channel::new_pin(p.PIN_27, Pull::None);
    // let mut p28 = Channel::new_pin(p.PIN_28, Pull::None);
    let ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    let stack = &*make_static!(Stack::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        make_static!(StackResources::<2>::new()),
        seed
    ));

    // Launch network task
    unwrap!(spawner.spawn(net_task(stack)));

    info!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    info!("IP address: {:?}", local_addr);

    fn make_app() -> picoserve::Router<AppRouter, AppState> {
        picoserve::Router::new()
            .route("/", get(|| async move { "Hello World" }))
            .route("/status", get(get_status))
    }

    let app = make_static!(make_app());

    let config = make_static!(picoserve::Config {
        start_read_request_timeout: Some(Duration::from_secs(5)),
        read_request_timeout: Some(Duration::from_secs(1)),
    });

    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];

    let shared_temp = SharedTempSensor(make_static!(Mutex::new(ts)));
    let shared_adc = SharedADC(make_static!(Mutex::new(adc)));

    loop {
        let id = 0;
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        log::info!("{id}: Listening on TCP:80...");
        if let Err(e) = socket.accept(80).await {
            log::warn!("{id}: accept error: {:?}", e);
            continue;
        }

        log::info!(
            "{id}: Received connection from {:?}",
            socket.remote_endpoint()
        );

        let (socket_rx, socket_tx) = socket.split();

        match picoserve::serve_with_state(
            app,
            EmbassyTimer,
            config,
            &mut [0; 2048],
            socket_rx,
            socket_tx,
            &AppState {
                temp_sensor: shared_temp,
                adc: shared_adc,
            },
        )
        .await
        {
            Ok(handled_requests_count) => {
                log::info!(
                    "{handled_requests_count} requests handled from {:?}",
                    socket.remote_endpoint()
                );
            }
            Err(err) => log::error!("{err:?}"),
        }
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

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
    let sign = if temp < 0.0 { -1.0 } else { 1.0 };
    let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
    (rounded_temp_x10 as f32) / 10.0
}
