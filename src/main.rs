use std::sync::{Mutex, Arc};
use std::time::Duration;
use windows::Devices::Sensors::{Accelerometer, Gyrometer};
use vigem_client::{DS4ReportExBuilder, DS4Buttons, DS4SpecialButtons};
use rusty_xinput as xi;
use xi::XInputState;

#[tokio::main]
async fn main() {
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
        Ok(h) => h,
        Err(e) => {
            eprintln!("Init XInput error: {:?}", e);
            return;
        }
    };

    print!("Getting gyro device... ");
    let gyro = match Gyrometer::GetDefault() {
        Ok(gyro) => {
            println!("Ok");
            gyro
        },
        Err(e) => {
            if e.code().0 == 0 {
                println!("Failed");
                eprintln!("No gyro found");
            } else {
                println!("Failed");
                eprintln!("Get gyro error: {}", e);
            }
            return;
        }
    };
    print!("Getting accelerometer device... ");
    let accel = match Accelerometer::GetDefault() {
        Ok(accel) => {
            println!("Ok");
            accel
        },
        Err(e) => {
            if e.code().0 == 0 {
                println!("Failed");
                eprintln!("No accelerometer found");
            } else {
                println!("Failed");
                eprintln!("Get accelerometer error: {}", e);
            }
            return;
        }
    };

    let xstate_g: Arc<Mutex<Option<XInputState>>> = Arc::new(Mutex::new(None));
    let gyro_data_g: Arc<Mutex<GyroData>> = Arc::new(Mutex::new(GyroData::default()));
    let accel_data_g: Arc<Mutex<AccelData>> = Arc::new(Mutex::new(AccelData::default()));

    let xstate_in = Arc::clone(&xstate_g);
    std::thread::spawn(move || {
        loop {
            *xstate_in.lock().expect("Unknown error 1") = match xi_handle.get_state(0) {
                Ok(s) => {
                    Some(s)
                }
                Err(e) => {
                    println!("No XInput device found, retrying: {:?}", e);
                    std::thread::sleep(Duration::from_millis(333));
                    None
                }
            };
            std::thread::sleep(Duration::from_millis(4)); // 250 Hz
        }
    });

    let gyro_data_in = Arc::clone(&gyro_data_g);
    std::thread::spawn(move || {
        loop {
            let gyro_data = read_gyro(&gyro).unwrap_or_default();
            let gyro_data = legion_go_gyro_axis_swap(gyro_data);
            (*gyro_data_in.lock().expect("Unknown error 1")).update(gyro_data);
        }
    });

    let accel_data_in = Arc::clone(&accel_data_g);
    std::thread::spawn(move || {
        loop {
            let accel_data = read_accel(&accel).unwrap_or_default();
            let accel_data = legion_go_accel_axis_swap(accel_data);
            (*accel_data_in.lock().expect("Unknown error 1")).update(accel_data);
        }
    });

    println!("Service started");
    loop {
        let xstate: Option<XInputState> = *xstate_g.lock().expect("Input process crashed");
        let gyro_data = *gyro_data_g.lock().expect("Input process crashed");
        let accel_data = *accel_data_g.lock().expect("Input process crashed");

        let report = put_xinput_state_into_builder(xstate, DS4ReportExBuilder::new());
        let report = report
            .gyro_x(convert_umdf_gyro_to_dualshock(gyro_data.x))
            .gyro_y(convert_umdf_gyro_to_dualshock(gyro_data.y))
            .gyro_z(convert_umdf_gyro_to_dualshock(gyro_data.z))
            .accel_x(convert_umdf_accel_to_dualshock(accel_data.x))
            .accel_y(convert_umdf_accel_to_dualshock(accel_data.y))
            .accel_z(convert_umdf_accel_to_dualshock(accel_data.z))
            .build();
        
        let _ = ds4wired.update_ex(&report);
        std::thread::sleep(Duration::from_millis(4));
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

        let special_buttons = DS4SpecialButtons::new()
            .touchpad(xstate.start_button());

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

fn convert_umdf_gyro_to_dualshock(umdf_value: f64) -> i16 {
    const SCALE_FACTOR: f64 = 20000.0;

    // Define the expected maximum value from the UMDF sensor
    let umdf_max: f64 = 500.0;

    // Clamp the umdf_value between -umdf_max and umdf_max
    let clamped_value = umdf_value.clamp(-umdf_max, umdf_max);

    // Now scale the clamped value to fit in the i16 range
    // We divide by UMDF's max to normalize it to a -1.0 to 1.0 range,
    // then we multiply by the maximum value i16 can hold.
    let scaled_value = clamped_value / umdf_max * SCALE_FACTOR;

    // Since i16 cannot hold fractional parts, we need to cast to i16
    // We use round to convert to the nearest integer to maintain as much precision as possible.
    scaled_value.round() as i16
}

fn convert_umdf_accel_to_dualshock(umdf_value: f64) -> i16 {
    const SCALE_FACTOR: f64 = 31900.0;
    let umdf_max: f64 = 4.2;
    let clamped_value = umdf_value.clamp(-umdf_max, umdf_max);
    let scaled_value = clamped_value / umdf_max * SCALE_FACTOR;
    scaled_value.round() as i16
}

#[derive(Debug, Clone, Copy)]
struct GyroData {
    x: f64,
    y: f64,
    z: f64,
}
impl Default for GyroData {
    fn default() -> Self {
        GyroData {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}
impl GyroData {
    fn update(&mut self, data: GyroData) {
        self.x = data.x;
        self.y = data.y;
        self.z = data.z;
    }
}

#[derive(Debug, Clone, Copy)]
struct AccelData {
    x: f64,
    y: f64,
    z: f64,
}
impl Default for AccelData {
    fn default() -> Self {
        AccelData {
            x: 0.983,
            y: 0.001,
            z: 0.001,
        }
    }
}
impl AccelData {
    fn update(&mut self, data: AccelData) {
        self.x = data.x;
        self.y = data.y;
        self.z = data.z;
    }
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
