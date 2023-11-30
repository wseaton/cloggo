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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_rp::flash::{Async, ERASE_SIZE, FLASH_BASE};
use embassy_rp::flash::{Flash, Mode};
use minicbor::decode::info;
use minicbor::{Decode, Decoder, Encode, Encoder};
use serde::{Deserialize, Serialize};

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

use ds::{CalibrationMatrix, CountingWriter};
use linreg::linear_regression_and_r_squared;

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
struct SharedFlash(
    &'static Mutex<
        CriticalSectionRawMutex,
        Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
    >,
);

#[derive(Clone, Copy)]
struct SharedCalibration(SharedCalibrationMatrix);

#[derive(Clone, Copy)]
struct SharedCalibrationStatus(&'static Mutex<CriticalSectionRawMutex, CalibrationStatus>);

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct CalibrationConstants {
    slope: I32F32,
    intercept: I32F32,
}

// Implement Encode and Decode for CalibrationConstants
impl Encode<()> for CalibrationConstants {
    fn encode<W: minicbor::encode::Write>(
        &self,
        e: &mut Encoder<W>,
        _: &mut (),
    ) -> Result<(), minicbor::encode::Error<W::Error>> {
        e.array(2)?
            .encode(self.slope.to_num::<f32>())?
            .encode(self.intercept.to_num::<f32>())?;
        Ok(())
    }
}

impl<'a> Decode<'a, ()> for CalibrationConstants {
    fn decode(d: &mut Decoder<'a>, _: &mut ()) -> Result<Self, minicbor::decode::Error> {
        d.array()?;
        let slope = I32F32::from_num(d.decode::<f32>()?);
        let intercept = I32F32::from_num(d.decode::<f32>()?);
        Ok(CalibrationConstants { slope, intercept })
    }
}

async fn write_calibration_data(
    flash: &mut Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
    constants: &CalibrationConstants,
    offset: u32,
) -> Result<(), embassy_rp::flash::Error> {
    if offset % ERASE_SIZE as u32 != 0 {
        debug!("Offset is not aligned to ERASE_SIZE");
        return Err(embassy_rp::flash::Error::Unaligned);
    }

    let mut buffer = [0u8; 64];
    let mut writer = CountingWriter::new(&mut buffer);
    let mut enc = Encoder::new(&mut writer);

    constants
        .encode(&mut enc, &mut ())
        .map_err(|_| embassy_rp::flash::Error::Other)?;
    debug!("Calibration constants encoded");

    let size = writer.position();
    if (offset as usize + size) > flash.capacity() {
        debug!("Data size and offset exceed flash capacity");
        return Err(embassy_rp::flash::Error::OutOfBounds);
    }

    flash.blocking_erase(offset, offset + ERASE_SIZE as u32)?;
    debug!("Flash sector erased");

    flash.blocking_write(offset, &buffer[..size])?;
    debug!("Calibration constants written to flash");

    Ok(())
}

// TODO: add some validation to the data read from flash
async fn read_calibration_data(
    flash: &mut Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
    offset: u32,
) -> Result<CalibrationConstants, embassy_rp::flash::Error> {
    defmt::debug!(
        "Attempting to read calibration data from flash at offset {:?}",
        offset
    );

    let mut buffer = [0u8; 64]; // Adjust the size accordingly
    match flash.blocking_read(offset, &mut buffer) {
        Ok(_) => defmt::debug!("Read operation successful."),
        Err(e) => {
            defmt::debug!("Failed to read from flash: {:?}", e);
            return Err(e);
        }
    };

    let mut dec = Decoder::new(&buffer[..]);
    match CalibrationConstants::decode(&mut dec, &mut ()) {
        Ok(constants) => {
            defmt::debug!("Decoding successful.");
            Ok(constants)
        }
        Err(_e) => {
            defmt::error!("Failed to decode calibration data");
            Err(embassy_rp::flash::Error::Other)
        }
    }
}

struct AppState {
    temp_sensor: SharedTempSensor,
    adc: SharedADC,
    pressure_sensor: SharedPressureSensor,
    calibration_matrix: SharedCalibration,
    calibration_status: SharedCalibrationStatus,
    flash: SharedFlash,
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
    Calibrated(CalibrationConstants),
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

impl picoserve::extract::FromRef<AppState> for SharedFlash {
    fn from_ref(state: &AppState) -> Self {
        state.flash
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
    State(SharedFlash(flash)): State<SharedFlash>,
) -> impl IntoResponse {
    // Immediately lock both resources
    let mut cal_status = calibration_status.lock().await;
    let cal_matrix = calibration_matrix.lock().await;

    // Depending on the calibration status, we may not need to do anything
    match *cal_status {
        CalibrationStatus::NotCalibrated => picoserve::response::Json(LinRegResponse {
            data_points: None,
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
            calibration_status: CalibrationStatus::NotCalibrated,
        }),
        CalibrationStatus::Calibrating(_) => picoserve::response::Json(LinRegResponse {
            data_points: None,
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
            calibration_status: CalibrationStatus::Calibrating(0),
        }),
        // For FullMeasurements and Calibrated, we proceed with calibration
        CalibrationStatus::FullMeasurements | CalibrationStatus::Calibrated(_) => {
            info!("Calibration matrix is full, calculating calibration constants");
            // allocate a new array of I32F32s for input to the linear regression function
            let mut data_points: [(I32F32, I32F32); 10] =
                [(I32F32::from_num(0), I32F32::from_num(0)); 10];

            // copy the calibration matrix into the new array
            for (i, (x, y)) in cal_matrix.iter().enumerate() {
                let x = I32F32::from_num(*x);
                let y = I32F32::from_num(*y);
                data_points[i] = (x, y);
            }

            // Now that we have the data, we can release the calibration matrix lock
            drop(cal_matrix);

            let (slope, intercept, r_squared) = linear_regression_and_r_squared(&data_points);
            info!(
                "Calibration constants calculated: slope = {}, intercept = {}, r_squared = {}",
                slope.to_num::<f32>(),
                intercept.to_num::<f32>(),
                r_squared.to_num::<f32>()
            );

            // set the calibration status to calibrated
            *cal_status = CalibrationStatus::Calibrated(CalibrationConstants { slope, intercept });

            info!("Writing the calibration constants to flash");
            let mut flash = flash.lock().await;
            match write_calibration_data(
                &mut flash,
                &CalibrationConstants { slope, intercept },
                ADDR_OFFSET,
            )
            .await
            {
                Ok(_) => info!("Calibration constants written to flash"),
                Err(e) => error!("Failed to write calibration constants to flash: {:?}", e),
            };

            // We can also release the calibration status lock before creating the response
            drop(cal_status);

            picoserve::response::Json(LinRegResponse {
                data_points: Some(data_points.map(|(x, y)| (x.to_num::<f32>(), y.to_num::<f32>()))),
                slope: slope.to_num::<f32>(),
                intercept: intercept.to_num::<f32>(),
                r_squared: r_squared.to_num::<f32>(),
                calibration_status: CalibrationStatus::Calibrated(CalibrationConstants {
                    slope,
                    intercept,
                }),
            })
        }
    }
}

async fn clear_calibration(
    State(SharedCalibration(calibration_matrix)): State<SharedCalibration>,
    State(SharedCalibrationStatus(calibration_status)): State<SharedCalibrationStatus>,
    State(SharedFlash(flash)): State<SharedFlash>,
) -> impl IntoResponse {
    {
        let mut cal_matrix = calibration_matrix.lock().await;
        cal_matrix.clear();
    }
    {
        *calibration_status.lock().await = CalibrationStatus::NotCalibrated;
    }

    // clear out the calibration data in flash
    info!("Clearing calibration data in flash");
    let mut flash = flash.lock().await;
    let constants = CalibrationConstants {
        slope: I32F32::from_num(0),
        intercept: I32F32::from_num(0),
    };
    match write_calibration_data(&mut flash, &constants, ADDR_OFFSET).await {
        Ok(_) => info!("Calibration constants written to flash"),
        Err(e) => error!("Failed to write calibration constants to flash: {:?}", e),
    };

    picoserve::response::Json(CalibrationStatus::NotCalibrated)
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

async fn get_pressure(
    State(SharedPressureSensor(pressure_sensor)): State<SharedPressureSensor>,
    State(SharedADC(adc)): State<SharedADC>,
    State(SharedCalibrationStatus(calibration_status)): State<SharedCalibrationStatus>,
) -> impl IntoResponse {
    // check calibration status
    let cs = *calibration_status.lock().await;
    if let CalibrationStatus::Calibrated(cal_constants) = cs {
        let res = {
            let mut pressure_channel = pressure_sensor.lock().await;
            adc.lock()
                .await
                .read(&mut pressure_channel)
                .await
                .unwrap_or(0)
        }; // Locks are dropped here because of the block scope.
           // Now process 'res' without holding any locks.
        let res = I32F32::from_num(res);
        let pressure = cal_constants.slope * res + cal_constants.intercept;
        picoserve::response::Json(PressureResponse {
            pressure: pressure.to_num::<f32>(),
            raw_reading: res.to_num::<u16>(),
            error: None,
        })
    } else {
        picoserve::response::Json(PressureResponse {
            pressure: 0.0,
            raw_reading: 0,
            error: Some(PressureResponseError {
                error: "Calibration not complete",
            }),
        })
    }
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
    };

    debug!(
        "Calibration reading: {} PSI (user input), {} ADC",
        read_psi, res
    );
    // add the reading to the calibration matrix
    {
        let mut cal_matrix = calibration_matrix.lock().await;

        cal_matrix.push((I32F32::from_num(res), I32F32::from_num(read_psi)));
        if !cal_matrix.is_full() {
            info!("Calibration matrix is not full yet");
            picoserve::response::Json(CalibrationStatus::Calibrating(cal_matrix.len() as u8))
        } else {
            info!("Calibration matrix is full, setting state to FullMeasurements");
            // if the calibration matrix is full, set the calibration status to calibrated
            *calibration_status.lock().await = CalibrationStatus::FullMeasurements;
            picoserve::response::Json(CalibrationStatus::FullMeasurements)
        }
    }
}

// Increasing this number causes Socket Stack exhaustion on the W5500
const WEB_TASK_POOL_SIZE: usize = 1;

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    id: usize,
    stack: &'static Stack<Device<'_>>,
    app: &'static picoserve::Router<AppRouter, AppState>,
    config: &'static picoserve::Config<Duration>,
    state: AppState,
) -> ! {
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];

    loop {
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
            &state,
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

    let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH3);

    let mac_addr = [0x02, 0x10, 0x10, 0x10, 0x10, 0x10];

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
            .route("/pressure", get(get_pressure))
    }

    let app = make_static!(make_app());

    let config = make_static!(picoserve::Config {
        start_read_request_timeout: Some(Duration::from_secs(5)),
        read_request_timeout: Some(Duration::from_secs(1)),
    });

    let shared_calibration_status =
        if let Ok(constants) = read_calibration_data(&mut flash, ADDR_OFFSET).await {
            info!("Calibration constants loaded");
            SharedCalibrationStatus(make_static!(Mutex::new(CalibrationStatus::Calibrated(
                constants
            ))))
        } else {
            info!("No calibration constants found");
            SharedCalibrationStatus(make_static!(Mutex::new(CalibrationStatus::NotCalibrated)))
        };

    let shared_flash = SharedFlash(make_static!(Mutex::new(flash)));
    let shared_temp = SharedTempSensor(make_static!(Mutex::new(ts)));
    let shared_adc = SharedADC(make_static!(Mutex::new(adc)));
    let shared_pressure = SharedPressureSensor(make_static!(Mutex::new(p26)));

    let calibration_matrix = SharedCalibration(make_static!(Mutex::new(CalibrationMatrix::new())));

    for id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(
            id,
            stack,
            app,
            config,
            AppState {
                temp_sensor: shared_temp,
                adc: shared_adc,
                pressure_sensor: shared_pressure,
                calibration_matrix,
                calibration_status: shared_calibration_status,
                flash: shared_flash,
            },
        ));
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
