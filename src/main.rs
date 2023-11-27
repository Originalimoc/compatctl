use std::sync::Arc;
use tokio::task::spawn_blocking;
use windows::Devices::Sensors::{Accelerometer, Gyrometer};
use vigem_client::DS4ReportExBuilder;

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

    print!("Getting gyro device... ");
    let gyro = Arc::new(match Gyrometer::GetDefault() {
        Ok(gyro) => {
            println!("Ok");
            gyro
        },
        Err(e) => {
            if e.code().0 == 0 {
                eprintln!("No gyro found");
            } else {
                eprintln!("Get gyro error: {}", e);
            }
            return;
        }
    });
    print!("Getting accelerometer device... ");
    let accel = Arc::new(match Accelerometer::GetDefault() {
        Ok(gyro) => {
            println!("Ok");
            gyro
        },
        Err(e) => {
            if e.code().0 == 0 {
                eprintln!("No accelerometer found");
            } else {
                eprintln!("Get accelerometer error: {}", e);
            }
            return;
        }
    });

    println!("Service started");
    loop {
        let sensor_data_read = read_sensor_aync(Arc::clone(&gyro), Arc::clone(&accel)).await.unwrap_or_default();
        let sensor_data_read = legion_go_axis_swap(sensor_data_read);
        let report = DS4ReportExBuilder::new()
            .gyro_x(convert_umdf_gyro_to_dualshock(sensor_data_read.g_x))
            .gyro_y(convert_umdf_gyro_to_dualshock(sensor_data_read.g_y))
            .gyro_z(convert_umdf_gyro_to_dualshock(sensor_data_read.g_z))
            .accel_x(convert_umdf_accel_to_dualshock(sensor_data_read.a_x))
            .accel_y(convert_umdf_accel_to_dualshock(sensor_data_read.a_y))
            .accel_z(convert_umdf_accel_to_dualshock(sensor_data_read.a_z))
            .build();
        let _ = ds4wired.update_ex(&report);
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

#[derive(Debug)]
struct SensorData {
    g_x: f64,
    g_y: f64,
    g_z: f64,
    a_x: f64,
    a_y: f64,
    a_z: f64,
}

impl Default for SensorData {
    fn default() -> Self {
        SensorData {
            g_x: 0.0,
            g_y: 0.0,
            g_z: 0.0,
            a_x: 0.0,
            a_y: 0.0,
            a_z: 0.0,
        }
    }
}

fn legion_go_axis_swap(raw: SensorData) -> SensorData {
    SensorData {
        g_x: -raw.g_x,
        g_y: raw.g_z,
        g_z: raw.g_y,
        a_x: raw.a_x,
        a_y: -raw.a_z,
        a_z: -raw.a_y,
    }
}

async fn read_sensor_aync(gyro: Arc<Gyrometer>, accel: Arc<Accelerometer>) -> windows::core::Result<SensorData> {
    let (gd, ad) = tokio::join!(read_gyro_async(gyro), read_accle_async(accel));
    let gd = gd.unwrap_or((0.0, 0.0, 0.0));
    let ad = ad.unwrap_or((0.0, 0.0, 0.0));
    Ok(SensorData {
        g_x: gd.0,
        g_y: gd.1,
        g_z: gd.2,
        a_x: ad.0,
        a_y: ad.1,
        a_z: ad.2,
    })
}

fn read_gyro(device: &Gyrometer) -> windows::core::Result<(f64, f64, f64)> {
    let reading = device.GetCurrentReading()?;
    Ok((reading.AngularVelocityX().unwrap_or(0.0), reading.AngularVelocityY().unwrap_or(0.0), reading.AngularVelocityZ().unwrap_or(0.0)))
}

async fn read_gyro_async(device: Arc<Gyrometer>) -> windows::core::Result<(f64, f64, f64)> {
    spawn_blocking(move || read_gyro(&device)).await.unwrap_or(Ok((0.0, 0.0, 0.0)))
}

fn read_accle(device: &Accelerometer) -> windows::core::Result<(f64, f64, f64)> {
    let reading = device.GetCurrentReading()?;
    Ok((reading.AccelerationX().unwrap_or(0.0), reading.AccelerationY().unwrap_or(0.0), reading.AccelerationZ().unwrap_or(0.0)))
}

async fn read_accle_async(device: Arc<Accelerometer>) -> windows::core::Result<(f64, f64, f64)> {
    spawn_blocking(move || read_accle(&device)).await.unwrap_or(Ok((0.0, 0.0, 0.0)))
}
