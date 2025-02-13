use std::sync::OnceLock;
use std::time::Duration;
use windows::Devices::Sensors::{Accelerometer, Gyrometer};
use vigem_client::{DS4ReportExBuilder, DS4Buttons, DS4SpecialButtons, DS4Status};
use rusty_xinput as xi;
use std::sync::{Arc, Mutex};
use tokio::time::interval;
use xi::XInputState;

static ENABLE_DS4_SHARE_BUTTON: OnceLock<bool> = OnceLock::new();

#[tokio::main]
async fn main() {
    for arg in std::env::args() {
        if arg.contains("--enable-share-button") {
            ENABLE_DS4_SHARE_BUTTON.get_or_init(|| true);
        }
    }
    ENABLE_DS4_SHARE_BUTTON.get_or_init(|| false);

    let Ok(vigem_driver_client) = vigem_client::Client::connect() else {
        eprintln!("Failed to connect to ViGEm Bus");
        return;
    };
    let mut ds4wired = vigem_client::DualShock4Wired::new(vigem_driver_client, vigem_client::TargetId::DUALSHOCK4_WIRED);
    if ds4wired.plugin().is_err() {
        eprintln!("Failed to init ViGEm virtual DS4 controller 1");
        return;
    }
    if ds4wired.wait_ready().is_err() {
        eprintln!("Failed to init ViGEm virtual DS4 controller 2");
        return;
    }

    let xi_handle = match xi::XInputHandle::load_default() {
        Ok(h) => std::sync::Arc::new(h),
        Err(e) => {
            eprintln!("Init XInput error: {:?}", e);
            return;
        }
    };

    let gyro = match Gyrometer::GetDefault() {
        Ok(gyro) => gyro,
        Err(e) => {
            eprintln!("Get gyro error: {}", e);
            return;
        }
    };
    let accel = match Accelerometer::GetDefault() {
        Ok(accel) => accel,
        Err(e) => {
            eprintln!("Get accelerometer error: {}", e);
            return;
        }
    };

    let xstate_mutex = Arc::new(Mutex::new(None));
    let gyro_mutex = Arc::new(Mutex::new(GyroData::default()));
    let accel_mutex = Arc::new(Mutex::new(AccelData::default()));

    let xi_handle_state_get = std::sync::Arc::clone(&xi_handle);
    let xstate_mutex_clone = Arc::clone(&xstate_mutex);
    tokio::spawn(async move {
        loop {
            let xstate = match xi_handle_state_get.get_state_ex(0) {
                Ok(s) => Some(s),
                Err(e) => {
                    println!("No XInput device found, retrying: {:?}", e);
                    None
                }
            };
            {
                let mut locked_xstate = xstate_mutex_clone.lock().unwrap();
                *locked_xstate = xstate;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    });

    let gyro_mutex_clone = Arc::clone(&gyro_mutex);
    tokio::spawn(async move {
        let inertia = 10.0;
        let mut previous_non_error_reading = GyroData::default();
        let mut broken_time = 0.0;
        loop {
            let mut og_gyro_data = read_gyro(&gyro).unwrap_or_default();
            og_gyro_data = legion_go_gyro_axis_swap(og_gyro_data);
            let x_is_broken = og_gyro_data.x < -124.1;
            let y_is_broken = og_gyro_data.y < -124.1;
            let z_is_broken = og_gyro_data.z < -124.1;
            let new_gyro_data = if x_is_broken || y_is_broken || z_is_broken {
                broken_time += 1.0;
                let previous_x_positive = previous_non_error_reading.x > 0.0;
                let previous_y_positive = previous_non_error_reading.y > 0.0;
                let previous_z_positive = previous_non_error_reading.z > 0.0;
                let workarounded_x = if x_is_broken { if previous_x_positive { 124.4 + inertia * broken_time } else { -124.4 - inertia * broken_time } } else { og_gyro_data.x };
                let workarounded_y = if y_is_broken { if previous_y_positive { 124.4 + inertia * broken_time } else { -124.4 - inertia * broken_time } } else { og_gyro_data.y };
                let workarounded_z = if z_is_broken { if previous_z_positive { 124.4 + inertia * broken_time } else { -124.4 - inertia * broken_time } } else { og_gyro_data.z };
                GyroData { x: workarounded_x, y: workarounded_y, z: workarounded_z }
            } else {
                broken_time = 0.0;
                previous_non_error_reading = og_gyro_data;
                og_gyro_data
            };
            {
                let mut locked_gyro = gyro_mutex_clone.lock().unwrap();
                *locked_gyro = new_gyro_data;
            }
        }
    });

    let accel_mutex_clone = Arc::clone(&accel_mutex);
    tokio::spawn(async move {
        loop {
            let mut accel_data = read_accel(&accel).unwrap_or_default();
            accel_data = legion_go_accel_axis_swap(accel_data);
            {
                let mut locked_accel = accel_mutex_clone.lock().unwrap();
                *locked_accel = accel_data;
            }
        }
    });

    let xi_handle_state_set = std::sync::Arc::clone(&xi_handle);
    ds4wired.request_notification().expect("already attached").spawn_thread(move |_, report| {
        let (lms, rms): (u16, u16) = (report.large_motor as u16, report.small_motor as u16);
        let (lms, rms) = (lms * 257, rms * 257);
        if let Err(e) = xi_handle_state_set.set_state(0, lms, rms) {
            println!("No XInput device found for rumble, retrying: {:?}", e);
            std::thread::sleep(Duration::from_millis(333));
        }
    });

    println!("Service started");

    let mut timestamp: u16 = 0;
    let mut interval = interval(Duration::from_micros(1250));

    loop {
        interval.tick().await;

        let xstate = {
            let locked_xstate = xstate_mutex.lock().unwrap();
            *locked_xstate
        };
        let gyro_data = {
            let locked_gyro = gyro_mutex.lock().unwrap();
            *locked_gyro
        };
        let accel_data = {
            let locked_accel = accel_mutex.lock().unwrap();
            *locked_accel
        };

        let report = put_xinput_state_into_builder(xstate, DS4ReportExBuilder::new())
            .gyro_x(convert_umdf_gyro_to_dualshock_x(gyro_data.x))
            .gyro_y(convert_umdf_gyro_to_dualshock_y(gyro_data.y))
            .gyro_z(convert_umdf_gyro_to_dualshock_z(gyro_data.z))
            .accel_x(convert_umdf_accel_to_dualshock(accel_data.x))
            .accel_y(convert_umdf_accel_to_dualshock(accel_data.y))
            .accel_z(convert_umdf_accel_to_dualshock(accel_data.z))
            .timestamp(timestamp)
            .status(DS4Status::with_battery_status(vigem_client::BatteryStatus::Full))
            .build();

        let _ = ds4wired.update_ex(&report);

        timestamp = timestamp.wrapping_add(188);
    }
}

fn put_xinput_state_into_builder(xstate: Option<XInputState>, ds4reb: DS4ReportExBuilder) -> DS4ReportExBuilder {
    if let Some(xstate) = xstate {
        let buttons = DS4Buttons::new()
            .triangle(xstate.north_button())
            .cross(xstate.south_button())
            .circle(xstate.east_button())
            .square(xstate.west_button())
            .dpad(map_dpad_directions(xstate.arrow_up(), xstate.arrow_down(), xstate.arrow_left(), xstate.arrow_right()))
            .options(xstate.select_button())
            .shoulder_left(xstate.left_shoulder())
            .shoulder_right(xstate.right_shoulder())
            .thumb_left(xstate.left_thumb_button())
            .thumb_right(xstate.right_thumb_button())
            .trigger_left(xstate.left_trigger_bool())
            .trigger_right(xstate.right_trigger_bool());

        let buttons = if *ENABLE_DS4_SHARE_BUTTON.get().unwrap_or(&false) {
            buttons.share(xstate.start_button())
        } else {
            buttons
        };

        let special_buttons = DS4SpecialButtons::new()
            .touchpad(xstate.start_button())
            .ps_home(xstate.guide_button());

        let (lx, ly) = xstate.left_stick_raw();
        let (rx, ry) = xstate.right_stick_raw();
        ds4reb
            .buttons(buttons)
            .special(special_buttons)
            .thumb_lx(normalize_i16_to_u8(lx, false))
            .thumb_ly(normalize_i16_to_u8(ly, true))
            .thumb_rx(normalize_i16_to_u8(rx, false))
            .thumb_ry(normalize_i16_to_u8(ry, true))
            .trigger_l(xstate.left_trigger())
            .trigger_r(xstate.right_trigger())
    } else {
        ds4reb
    }
}

fn normalize_i16_to_u8(value: i16, flip: bool) -> u8 {
    if flip {
        255 - (((value as f64 + 32768.0) / 65535.0 * 255.0).round() as u8)
    } else {
        ((value as f64 + 32768.0) / 65535.0 * 255.0).round() as u8
    }
}

fn map_dpad_directions(up: bool, down: bool, left: bool, right: bool) -> vigem_client::DpadDirection {
    match (up, down, left, right) {
        (true, false, false, false) => vigem_client::DpadDirection::North,
        (true, false, false, true) => vigem_client::DpadDirection::NorthEast,
        (false, false, false, true) => vigem_client::DpadDirection::East,
        (false, true, false, true) => vigem_client::DpadDirection::SouthEast,
        (false, true, false, false) => vigem_client::DpadDirection::South,
        (false, true, true, false) => vigem_client::DpadDirection::SouthWest,
        (false, false, true, false) => vigem_client::DpadDirection::West,
        (true, false, true, false) => vigem_client::DpadDirection::NorthWest,
        _ => vigem_client::DpadDirection::None,
    }
}

fn convert_umdf_gyro_to_dualshock_x(umdf_value: f64) -> i16 {
    let scale: f64 = 1.0f64;
    let intermediate_value = convert_umdf_gyro_to_dualshock(umdf_value) as f64 * scale;
     // Define i16 min and max to avoid magic numbers
    const I16_MAX: f64 = 32767.0;
    const I16_MIN: f64 = -32768.0;
    intermediate_value.clamp(I16_MIN, I16_MAX) as i16
}
fn convert_umdf_gyro_to_dualshock_y(umdf_value: f64) -> i16 {
    let scale: f64 = 1.0f64;
    let intermediate_value = convert_umdf_gyro_to_dualshock(umdf_value) as f64 * scale;
     // Define i16 min and max to avoid magic numbers
    const I16_MAX: f64 = 32767.0;
    const I16_MIN: f64 = -32768.0;
    intermediate_value.clamp(I16_MIN, I16_MAX) as i16
}
fn convert_umdf_gyro_to_dualshock_z(umdf_value: f64) -> i16 {
    let scale: f64 = 1.0f64;
    let intermediate_value = convert_umdf_gyro_to_dualshock(umdf_value) as f64 * scale;
     // Define i16 min and max to avoid magic numbers
    const I16_MAX: f64 = 32767.0;
    const I16_MIN: f64 = -32768.0;
    intermediate_value.clamp(I16_MIN, I16_MAX) as i16
}
fn convert_umdf_gyro_to_dualshock(umdf_value: f64) -> i16 {
    // Define the maximum angular velocity representable by the DualShock 4 gyro.
    const MAX_DPS: f64 = 2000.0;
    // Define i16 min and max to avoid magic numbers
    const I16_MAX: f64 = 32767.0;
    const I16_MIN: f64 = -32768.0;

    // 1. Clamping
    let clamped_value = umdf_value.clamp(-MAX_DPS, MAX_DPS);

    // 2. Scaling and 3. Rounding and Type Conversion
    // Calculate the sensitivity.  We are going from degrees/second to i16.
    let scale_factor: f64 = I16_MAX / MAX_DPS;
    let scaled_value = (clamped_value * scale_factor).round();

    //clamp to i16 range, cast to i16 and return.
    scaled_value.clamp(I16_MIN, I16_MAX) as i16
}

fn convert_umdf_accel_to_dualshock(umdf_value: f64) -> i16 {
    const SCALE_FACTOR: f64 = 84626.0;
    let umdf_max: f64 = 9.8;
    let clamped_value = umdf_value.clamp(-umdf_max, umdf_max);
    let scaled_value = clamped_value / umdf_max * SCALE_FACTOR;
    scaled_value.round() as i16
}

#[derive(Debug, Clone, Copy, Default)]
struct GyroData {
    x: f64,
    y: f64,
    z: f64,
}

impl std::ops::Add for GyroData {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}
impl std::ops::Div<f64> for GyroData {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct AccelData {
    x: f64,
    y: f64,
    z: f64,
}

fn legion_go_gyro_axis_swap(raw: GyroData) -> GyroData {
    GyroData {
        x: -raw.x,
        y: raw.z,
        z: raw.y,
    }
}

fn legion_go_accel_axis_swap(raw: AccelData) -> AccelData {
    AccelData {
        x: raw.x,
        y: -raw.z,
        z: -raw.y,
    }
}

fn read_gyro(device: &Gyrometer) -> windows::core::Result<GyroData> {
    let reading = device.GetCurrentReading()?;
    let data = GyroData {
        x: reading.AngularVelocityX().unwrap_or(0.0),
        y: reading.AngularVelocityY().unwrap_or(0.0),
        z: reading.AngularVelocityZ().unwrap_or(0.0),
    };
    Ok(data)
}

fn read_accel(device: &Accelerometer) -> windows::core::Result<AccelData> {
    let reading = device.GetCurrentReading()?;
    Ok(AccelData {
        x: reading.AccelerationX().unwrap_or(0.0),
        y: reading.AccelerationY().unwrap_or(0.0),
        z: reading.AccelerationZ().unwrap_or(0.0),
    })
}
