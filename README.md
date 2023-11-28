# compatctl
Unify input and convert controller type, currently emulating DS4 via ViGEmBus

# Supported device
Legion Go

# Todo
DirectInput(For more button, but no existing working lib)

# Build and Run
Just `cargo build --release` on Windows(or use target), need ViGEmBus driver to run.

# Known issue
Motion sensors stop working after device sleep, but SensorExplorer also stops reading data. So this is either a Lenovo or AMD driver problem, I can't fix it.
