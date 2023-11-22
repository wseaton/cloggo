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

use fixed::types::{I16F16, I32F32};
use picoserve::extract::State;
use picoserve::response::IntoResponse;
use picoserve::routing::{get, parse_path_segment, post};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;

        ADC_IRQ_FIFO => InterruptHandler;

});

use rand::RngCore;

use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

mod ds;
mod linreg;

use ds::CalibrationMatrix;
use linreg::linear_regression_and_r_squared;

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

type SharedSensor = &'static Mutex<CriticalSectionRawMutex, Channel<'static>>;
type SharedCalibrationMatrix = &'static Mutex<CriticalSectionRawMutex, CalibrationMatrix>;

#[derive(Clone, Copy)]
struct SharedTempSensor(SharedSensor);

#[derive(Clone, Copy)]
struct SharedPressureSensor(SharedSensor);

#[derive(Clone, Copy)]
struct SharedADC(&'static Mutex<CriticalSectionRawMutex, Adc<'static, embassy_rp::adc::Async>>);

#[derive(Clone, Copy)]
struct SharedCalibration(SharedCalibrationMatrix);

#[derive(Clone, Copy)]
struct SharedCalibrationStatus(&'static Mutex<CriticalSectionRawMutex, CalibrationStatus>);

struct AppState {
    temp_sensor: SharedTempSensor,
    adc: SharedADC,
    pressure_sensor: SharedPressureSensor,
    calibration_matrix: SharedCalibration,
    calibration_status: SharedCalibrationStatus,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
enum CalibrationStatus {
    // The calibration process has not started yet
    NotCalibrated,
    // The calbiration process is ongoing. The u8 is the number of data points collected so far.
    Calibrating(u8),
    // The calibration matrix is full, but the calibration process is not done yet.
    FullMeasurements,
    // Calibration is done (eg. constants have been calculated)
    Calibrated,
}

type AppRouter = impl picoserve::routing::PathRouter<AppState>;

impl picoserve::extract::FromRef<AppState> for SharedTempSensor {
    fn from_ref(state: &AppState) -> Self {
        state.temp_sensor
    }
}

impl picoserve::extract::FromRef<AppState> for SharedCalibration {
    fn from_ref(state: &AppState) -> Self {
        state.calibration_matrix
    }
}

impl picoserve::extract::FromRef<AppState> for SharedPressureSensor {
    fn from_ref(state: &AppState) -> Self {
        state.pressure_sensor
    }
}

impl picoserve::extract::FromRef<AppState> for SharedADC {
    fn from_ref(state: &AppState) -> Self {
        state.adc
    }
}

impl picoserve::extract::FromRef<AppState> for SharedCalibrationStatus {
    fn from_ref(state: &AppState) -> Self {
        state.calibration_status
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

#[derive(Clone, serde::Serialize)]
struct LinRegResponse {
    // skip serializing this field if it's None
    #[serde(skip_serializing_if = "Option::is_none")]
    data_points: Option<[(f32, f32); 10]>,
    slope: f32,
    intercept: f32,
    r_squared: f32,
    calibration_status: CalibrationStatus,
}

async fn get_linreg(
    State(SharedCalibration(calibration_matrix)): State<SharedCalibration>,
    State(SharedCalibrationStatus(calibration_status)): State<SharedCalibrationStatus>,
) -> impl IntoResponse {
    {
        match *calibration_status.lock().await {
            CalibrationStatus::NotCalibrated => {
                return picoserve::response::Json(LinRegResponse {
                    data_points: None,
                    slope: 0.0,
                    intercept: 0.0,
                    r_squared: 0.0,
                    calibration_status: CalibrationStatus::NotCalibrated,
                });
            }
            CalibrationStatus::Calibrating(_) => {
                return picoserve::response::Json(LinRegResponse {
                    data_points: None,
                    slope: 0.0,
                    intercept: 0.0,
                    r_squared: 0.0,
                    calibration_status: CalibrationStatus::Calibrating(0),
                });
            }
            // in these cases we want to continue with the calibration
            CalibrationStatus::FullMeasurements => {}
            CalibrationStatus::Calibrated => {}
        }
    }
    // allocate a new array of I32F32s for input to the linear regression function
    let mut data_points: [(I32F32, I32F32); 10] = [(I32F32::from_num(0), I32F32::from_num(0)); 10];
    // scoping here to not hold the lock for longer than necessary
    {
        let cal_matrix = calibration_matrix.lock().await;

        // copy the calibration matrix into the new array
        for (i, (x, y)) in cal_matrix.iter().enumerate() {
            let x = I32F32::from_num(*x);
            let y = I32F32::from_num(*y);
            data_points[i] = (x, y);
        }
    }

    let (slope, intercept, r_squared) = linear_regression_and_r_squared(&data_points);

    {
        // set the calibration status to calibrated
        *calibration_status.lock().await = CalibrationStatus::Calibrated;
    }

    picoserve::response::Json(LinRegResponse {
        data_points: Some(data_points.map(|(x, y)| (x.to_num::<f32>(), y.to_num::<f32>()))),
        slope: slope.to_num::<f32>(),
        intercept: intercept.to_num::<f32>(),
        r_squared: r_squared.to_num::<f32>(),
        calibration_status: CalibrationStatus::Calibrated,
    })
}

async fn clear_calibration(
    State(SharedCalibration(calibration_matrix)): State<SharedCalibration>,
    State(SharedCalibrationStatus(calibration_status)): State<SharedCalibrationStatus>,
) -> impl IntoResponse {
    {
        let mut cal_matrix = calibration_matrix.lock().await;
        cal_matrix.clear();
    }
    {
        *calibration_status.lock().await = CalibrationStatus::NotCalibrated;
    }
    picoserve::response::Json(CalibrationStatus::NotCalibrated)
}

async fn calibrate_reading(
    read_psi: u16,
    State(SharedCalibration(calibration_matrix)): State<SharedCalibration>,
    State(SharedCalibrationStatus(calibration_status)): State<SharedCalibrationStatus>,
    State(SharedPressureSensor(pressure_sensor)): State<SharedPressureSensor>,
    State(SharedADC(adc)): State<SharedADC>,
) -> impl IntoResponse {
    // get the voltage reading of the pressure sensor
    let res = {
        let mut pressure_channel = pressure_sensor.lock().await;
        adc.lock()
            .await
            .read(&mut pressure_channel)
            .await
            .unwrap_or(0)
    }; // Locks are dropped here because of the block scope.
       // Now process 'res' without holding any locks.

    // add the reading to the calibration matrix
    {
        let mut cal_matrix = calibration_matrix.lock().await;

        cal_matrix.push((I32F32::from_num(res), I32F32::from_num(read_psi)));
        if !cal_matrix.is_full() {
            picoserve::response::Json(CalibrationStatus::Calibrating(cal_matrix.len() as u8))
        } else {
            // if the calibration matrix is full, set the calibration status to calibrated
            *calibration_status.lock().await = CalibrationStatus::FullMeasurements;
            picoserve::response::Json(CalibrationStatus::FullMeasurements)
        }
    }
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
            .route("/linreg", get(get_linreg))
            .route("/clear_calibration", post(clear_calibration))
            .route(
                ("/calibrate_reading", parse_path_segment::<u16>()),
                post(calibrate_reading),
            )
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
    let shared_pressure = SharedPressureSensor(make_static!(Mutex::new(p26)));

    let calibration_matrix = SharedCalibration(make_static!(Mutex::new(CalibrationMatrix::new())));
    let shared_calibration_status =
        SharedCalibrationStatus(make_static!(Mutex::new(CalibrationStatus::NotCalibrated)));

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
                pressure_sensor: shared_pressure,
                calibration_matrix,
                calibration_status: shared_calibration_status,
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
