use std::sync::OnceLock;
use std::time::Duration;
use windows::Devices::Sensors::{Accelerometer, Gyrometer};
use vigem_client::{DS4ReportExBuilder, DS4Buttons, DS4SpecialButtons, DS4Status};
use rusty_xinput as xi;
use xi::XInputState;
use tokio::sync::mpsc;

static ENABLE_DS4_SHARE_BUTTON: OnceLock<bool> = OnceLock::new();

const GYRO_SCALE_FACTOR: f64 = 50.0; // Adjust as needed
const ACCEL_SCALE_FACTOR: f64 = 8560.0; // DS4 accel is about -4g to 4g

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

    let (xstate_tx, mut xstate_rx) = mpsc::channel::<Option<XInputState>>(3);
    let (gyro_tx, mut gyro_rx) = mpsc::channel::<GyroData>(3);
    let (accel_tx, mut accel_rx) = mpsc::channel::<AccelData>(3);

    let xi_handle_state_get = std::sync::Arc::clone(&xi_handle);
    tokio::spawn(async move {
        loop {
            let xstate = match xi_handle_state_get.get_state_ex(0) {
                Ok(s) => Some(s),
                Err(e) => {
                    println!("No XInput device found, retrying: {:?}", e);
                    tokio::time::sleep(Duration::from_millis(333)).await;
                    None
                }
            };
            if xstate_tx.send(xstate).await.is_err() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(3)).await;
        }
    });

    tokio::spawn(async move {
        loop {
            let mut gyro_data = read_gyro(&gyro).unwrap_or_default();
            gyro_data = legion_go_gyro_axis_swap(gyro_data);

            if gyro_tx.send(gyro_data).await.is_err() {
                break;
            }
        }
    });

    tokio::spawn(async move {
        loop {
            let mut accel_data = read_accel(&accel).unwrap_or_default();
            accel_data = legion_go_accel_axis_swap(accel_data);

            if accel_tx.send(accel_data).await.is_err() {
                break;
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
    let mut last_update_time = quanta::Instant::now();

    loop {
        let xstate = xstate_rx.recv().await.unwrap_or(None);
        let gyro_data = gyro_rx.recv().await.unwrap_or_default();
        let accel_data = accel_rx.recv().await.unwrap_or_default();

        // Convert gyro data to scaled DS4 gyro values
        let gyro_x = (gyro_data.x * GYRO_SCALE_FACTOR) as i16;
        let gyro_y = (gyro_data.y * GYRO_SCALE_FACTOR) as i16;
        let gyro_z = (gyro_data.z * GYRO_SCALE_FACTOR) as i16;

        // Convert accel data to scaled DS4 accel values
        let accel_x = (accel_data.x * ACCEL_SCALE_FACTOR) as i16;
        let accel_y = (accel_data.y * ACCEL_SCALE_FACTOR) as i16;
        let accel_z = (accel_data.z * ACCEL_SCALE_FACTOR) as i16;

        let report = put_xinput_state_into_builder(xstate, DS4ReportExBuilder::new())
            .gyro_x(gyro_x)
            .gyro_y(gyro_y)
            .gyro_z(gyro_z)
            .accel_x(accel_x)
            .accel_y(accel_y)
            .accel_z(accel_z)
            .timestamp(timestamp)
            .status(DS4Status::with_battery_status(vigem_client::BatteryStatus::Full))
            .build();

        let _ = ds4wired.update_ex(&report);

        let now = quanta::Instant::now();
        let elapsed = now - last_update_time;
        last_update_time = now;

        // Calculate the timestamp increment proportionally to 1.25ms per 188 units.
        let elapsed_ms = elapsed.as_secs_f64() * 1000.0;
        let timestamp_increment = (elapsed_ms * (188.0 / 1.25)).round() as u16;
        timestamp = timestamp.wrapping_add(timestamp_increment);
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
    Ok(GyroData {
        x: reading.AngularVelocityX().unwrap_or(0.0),
        y: reading.AngularVelocityY().unwrap_or(0.0),
        z: reading.AngularVelocityZ().unwrap_or(0.0),
    })
}

fn read_accel(device: &Accelerometer) -> windows::core::Result<AccelData> {
    let reading = device.GetCurrentReading()?;
    Ok(AccelData {
        x: reading.AccelerationX().unwrap_or(0.0),
        y: reading.AccelerationY().unwrap_or(0.0),
        z: reading.AccelerationZ().unwrap_or(0.0),
    })
}
