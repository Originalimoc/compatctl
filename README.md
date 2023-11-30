# compatctl
Unify input and convert controller type, currently emulating DS4 via ViGEmBus

# Supported device
Legion Go

# Todo
More button?

# Build
Just `cargo build --release` on Windows(or use target)

# Run
Need ViGEmBus driver to run
Support CLI argument --enable-share-button

# Known issue
Motion sensors stop working after device sleep, but SensorExplorer also stops reading data. So this is either a Lenovo or AMD driver problem, I can't fix it.
