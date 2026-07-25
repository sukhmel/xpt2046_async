#![doc(html_root_url = "https://docs.rs/xpt2046-async")]
#![doc(issue_tracker_base_url = "https://github.com/sukhmel/xpt2046_async/issues/")]
#![deny(
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    unused_variables,
    unreachable_code,
    unused_comparisons,
    unused_must_use
)]
#![warn(clippy::pedantic, clippy::nursery, clippy::all, clippy::cargo)]
#![no_std]

//! A platform agnostic Rust driver for XPT2046 touch controller, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!

pub use crate::{
    calibration::CalibrationPoint,
    error::{BusError, Error},
};
use core::{fmt::Debug, ops::RemAssign};
use embedded_graphics_core::geometry::Point;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::digital::Wait;
use embedded_hal::{delay::DelayNs, digital::InputPin};

#[cfg(feature = "calibration")]
use embedded_graphics_core::pixelcolor::Rgb565;
#[cfg(feature = "calibration")]
use embedded_graphics_core::prelude::{DrawTarget, RgbColor};
#[cfg(feature = "calibration")]
use crate::calibration::{calculate_calibration, calibration_draw_point};

pub mod calibration;
pub mod error;

const CHANNEL_SETTING_X: u8 = 0b1001_0000;
const CHANNEL_SETTING_Y: u8 = 0b1101_0000;

const MAX_SAMPLES: usize = 2; //:D/
const TX_BUFF_LEN: usize = 5;

#[derive(Debug, Clone, Copy)]
pub struct CalibrationData {
    pub alpha_x: f32,
    pub beta_x: f32,
    pub delta_x: f32,
    pub alpha_y: f32,
    pub beta_y: f32,
    pub delta_y: f32,
}

/// Orientation of the touch screen
#[derive(Debug, Clone, Copy)]
pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

impl Orientation {
    /// Default location for the test touch point
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    #[must_use]
    pub const fn calibration_point(&self) -> CalibrationPoint {
        match self {
            Self::Portrait | Self::PortraitFlipped => CalibrationPoint {
                a: Point::new(10, 10),
                b: Point::new(80, 210),
                c: Point::new(200, 170),
            },
            Self::Landscape | Self::LandscapeFlipped => CalibrationPoint {
                a: Point::new(20, 25),
                b: Point::new(160, 220),
                c: Point::new(300, 110),
            },
        }
    }

    /// Default calibration values used for calculating the touch points
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    #[must_use]
    pub const fn calibration_data(&self) -> CalibrationData {
        match self {
            Self::Portrait => CalibrationData {
                alpha_x: -0.000_933_7,
                beta_x: -0.063_683_9,
                delta_x: 250.342,
                alpha_y: -0.088_977_5,
                beta_y: -0.001_181_10,
                delta_y: 356.538,
            },
            Self::PortraitFlipped => CalibrationData {
                alpha_x: 0.000_610_0,
                beta_x: 0.064_782_8,
                delta_x: -13.634,
                alpha_y: 0.089_060_9,
                beta_y: 0.000_138_1,
                delta_y: -35.73,
            },
            Self::Landscape => CalibrationData {
                alpha_x: -0.088_554_2,
                beta_x: 0.001_653_2,
                delta_x: 349.800,
                alpha_y: 0.000_730_9,
                beta_y: 0.065_436_99,
                delta_y: -15.290,
            },
            Self::LandscapeFlipped => CalibrationData {
                alpha_x: 0.090_221_6,
                beta_x: 0.000_651_0,
                delta_x: -38.657,
                alpha_y: -0.001_000_5,
                beta_y: -0.066_703_0,
                delta_y: 258.08,
            },
        }
    }
}

/// Current state of the driver
#[derive(PartialEq, Eq, Debug)]
pub enum TouchScreenState {
    /// Driver waith for touch
    IDLE,
    /// Driver debounces the touch
    PRESAMPLING,
    /// Confirmed touch
    TOUCHED,
    /// Touch released
    RELEASED,
}

#[derive(Debug, PartialEq, Eq)]
pub enum TouchScreenOperationMode {
    /// Normal touch reading
    NORMAL,
    /// Manual calibration mode
    CALIBRATION,
}

#[derive(Debug)]
pub struct TouchSamples {
    /// All the touch samples
    samples: [Point; MAX_SAMPLES],
    /// current number of captured samples
    counter: usize,
}

impl Default for TouchSamples {
    fn default() -> Self {
        Self {
            counter: 0,
            samples: [Point::default(); MAX_SAMPLES],
        }
    }
}

impl TouchSamples {
    #[must_use]
    pub fn average(&self) -> Point {
        let mut x = 0;
        let mut y = 0;

        for point in self.samples {
            x += point.x;
            y += point.y;
        }
        x /= i32::try_from(MAX_SAMPLES).unwrap_or(i32::MAX);
        y /= i32::try_from(MAX_SAMPLES).unwrap_or(i32::MAX);
        Point::new(x, y)
    }
}

#[derive(Debug)]
pub struct Xpt2046<SPI, PinIRQ> {
    /// The SPI interface
    spi: SPI,
    /// Interrupt control pin
    irq: PinIRQ,
    /// Internall buffers tx
    tx_buff: [u8; TX_BUFF_LEN],
    /// Internal buffer for rx
    rx_buff: [u8; TX_BUFF_LEN],
    /// Current driver state
    screen_state: TouchScreenState,
    /// Buffer for the touch data samples
    ts: TouchSamples,
    calibration_data: CalibrationData,
    operation_mode: TouchScreenOperationMode,
    /// Location of the touch points used for
    /// performing manual calibration
    #[cfg(feature = "calibration")]
    calibration_point: CalibrationPoint,
}

impl<SPI, PinIRQ> Xpt2046<SPI, PinIRQ>
where
    SPI: SpiDevice<u8>,
    PinIRQ: InputPin,
{
    pub fn new(spi: SPI, irq: PinIRQ, orientation: Orientation) -> Self {
        Self {
            spi,
            irq,
            tx_buff: [0; TX_BUFF_LEN],
            rx_buff: [0; TX_BUFF_LEN],
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_data: orientation.calibration_data(),
            operation_mode: TouchScreenOperationMode::NORMAL,
            #[cfg(feature = "calibration")]
            calibration_point: orientation.calibration_point(),
        }
    }
}

impl<SPI, PinIRQ, SPIError, CSError> Xpt2046<SPI, PinIRQ>
where
    SPI: SpiDevice<u8, Error = SPIError>,
    // `InputPin` is used to sample the level while a touch is in progress;
    // `Wait` lets us sleep (instead of polling) until a touch begins.
    PinIRQ: InputPin<Error = CSError> + Wait,
    SPIError: Debug,
    CSError: Debug,
{
    async fn spi_read(&mut self) -> Result<(), Error<BusError<SPIError, CSError>>> {
        self.spi
            .transfer(&mut self.rx_buff, &self.tx_buff)
            .await
            .map_err(|e| Error::Bus(BusError::Spi(e)))?;
        Ok(())
    }

    /// Read raw values from the XPT2046 driver
    async fn read_xy(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        self.spi_read().await?;

        let x = (i32::from(self.rx_buff[1]) << 8) | i32::from(self.rx_buff[2]);
        let y = (i32::from(self.rx_buff[3]) << 8) | i32::from(self.rx_buff[4]);
        Ok(Point::new(x, y))
    }

    /// Read the calibrated point of touch from XPT2046
    async fn read_touch_point(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        let raw_point = self.read_xy().await?;

        let (x, y) = match self.operation_mode {
            TouchScreenOperationMode::NORMAL => {
                #[allow(clippy::cast_precision_loss)]
                let x = self.calibration_data.alpha_x * raw_point.x as f32
                    + self.calibration_data.beta_x * raw_point.y as f32
                    + self.calibration_data.delta_x;
                #[allow(clippy::cast_precision_loss)]
                let y = self.calibration_data.alpha_y * raw_point.x as f32
                    + self.calibration_data.beta_y * raw_point.y as f32
                    + self.calibration_data.delta_y;
                #[allow(clippy::cast_possible_truncation)]
                (x as i32, y as i32)
            }
            TouchScreenOperationMode::CALIBRATION => {
                /*
                 * We're running calibration so just return raw
                 * point measurements without compensation
                 */
                (raw_point.x, raw_point.y)
            }
        };
        Ok(Point::new(x, y))
    }

    /// Get the actual touch point
    pub fn get_touch_point(&self) -> Point {
        self.ts.average()
    }

    /// Check if the display is currently touched
    pub fn is_touched(&self) -> bool {
        self.screen_state == TouchScreenState::TOUCHED
    }

    /// Sometimes the TOUCHED state needs to be cleared
    pub const fn clear_touch(&mut self) {
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    /// Reset the driver and preload tx buffer with register data.
    ///
    /// # Errors
    ///
    /// If SPI read fails.
    pub async fn init<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<BusError<SPIError, CSError>>> {
        self.tx_buff[0] = 0x80;
        self.spi_read().await?;
        delay.delay_ms(1);

        /*
         * Load the tx_buffer with the channels config
         * for all subsequent reads
         * The byte shifting provides padding to align the read bytes with the
         * DCLK. XPT2046 datasheet figure 12
         */
        self.tx_buff = [
            CHANNEL_SETTING_X >> 3,
            CHANNEL_SETTING_X << 5,
            CHANNEL_SETTING_Y >> 3,
            CHANNEL_SETTING_Y << 5,
            0,
        ];
        Ok(())
    }

    /// Continually runs and and collects the touch data from xpt2046.
    /// You should drive this either in some main loop or dedicated timer
    /// interrupt
    ///
    /// # Errors
    ///
    /// If SPI read fails, IRQ pin read fails, or waiting for IRQ low state returned an error.
    pub async fn run(
        &mut self,
    ) -> Result<(), Error<BusError<SPIError, CSError>>> {
        if self.screen_state == TouchScreenState::IDLE {
            self.irq.wait_for_low().await?;
            self.screen_state = TouchScreenState::PRESAMPLING;
            return Ok(());
        }

        let point = self.read_touch_point().await?;
        let is_low = self.irq.is_low()?;
        match self.screen_state {
            // Handled above by awaiting the IRQ; should be unreachable here.
            TouchScreenState::IDLE => {}
            TouchScreenState::PRESAMPLING => {
                if !is_low {
                    self.screen_state = TouchScreenState::RELEASED;
                }
                let point_sample = point;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter + 1 == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = point;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                /*
                 * Wrap around the counter if the screen
                 * is touched for longer time
                 */
                self.ts.counter.rem_assign(MAX_SAMPLES - 1);
                if !is_low {
                    self.screen_state = TouchScreenState::RELEASED;
                }
            }
            TouchScreenState::RELEASED => {
                self.screen_state = TouchScreenState::IDLE;
                self.ts.counter = 0;
            }
        }
        Ok(())
    }

    /// Collects the reading for 3 sample points and
    /// calculates a set of calibration data. The default calibration data seem
    /// to work ok but if for some reason touch screen needs to be recalibrated
    /// then look no further.
    /// This should be run after init() method.
    #[cfg(feature = "calibration")]
    pub async fn calibrate<DT, DELAY>(
        &mut self,
        dt: &mut DT,
        delay: &mut DELAY,
    ) -> Result<CalibrationData, Error<BusError<SPIError, CSError>>>
    where
        DT: DrawTarget<Color = Rgb565>,
        DELAY: DelayNs,
    {
        let mut calibration_count = 0;
        let mut retry = 3;
        let mut new_a = Point::zero();
        let mut new_b = Point::zero();
        let mut new_c = Point::zero();
        let old_cp = self.calibration_point.clone();
        // Prepare the screen for points
        let _ = dt.clear(Rgb565::BLACK);

        // Set correct state to fetch raw data from touch controller
        self.operation_mode = TouchScreenOperationMode::CALIBRATION;
        while calibration_count < 4 {
            match calibration_count {
                0 => {
                    calibration_draw_point(dt, &old_cp.a);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_a = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                1 => {
                    calibration_draw_point(dt, &old_cp.b);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_b = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                2 => {
                    calibration_draw_point(dt, &old_cp.c);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_c = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                3 => {
                    // Create new calibration point from the captured samples
                    self.calibration_point = CalibrationPoint {
                        a: new_a,
                        b: new_b,
                        c: new_c,
                    };
                    // and then re-calculate calibration
                    match calculate_calibration(&old_cp, &self.calibration_point) {
                        Ok(new_calibration_data) => {
                            self.calibration_data = new_calibration_data;
                            calibration_count += 1;
                        }
                        Err(e) => {
                            // We have problem calculating new values
                            if retry == 0 {
                                return Err(Error::Calibration(e));
                            }
                            // If our calculation failed let's retry
                            retry -= 1;
                            calibration_count = 0;

                            let _ = dt.clear(Rgb565::BLACK);
                        }
                    }
                }
                _ => {}
            }
            // We must run our state machine to capture user input, but we do it after drawing
            if calibration_count < 4 {
                self.run().await?;
            }
        }

        let _ = dt.clear(Rgb565::WHITE);
        self.operation_mode = TouchScreenOperationMode::NORMAL;

        Ok(self.calibration_data)
    }
}
