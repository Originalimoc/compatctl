# compatctl
Unify input and convert controller type, currently emulating DS4 via ViGEmBus

# Supported device
Legion Go

# Todo
merge with XInput/DirectInput

# Build
Just `cargo build --release` on Windows

# Known issue
Stop working after device sleep, but SensorExplorer also stops reading data. So this is either a Lenovo or AMD problem, I can't fix it.
