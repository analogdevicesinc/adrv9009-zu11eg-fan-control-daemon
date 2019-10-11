## FAN Control

This is a system daemon for controlling the fan speed on the ADRV9009-ZU11EG System On Module.

### Dependencies

Make sure **libiio** is installed on your system. It can be installed using most of the package-managers out there.

### Building

Run:

```bash
cmake .
make
```

To support typical init services (systemd, upstart and sysv), run `cmake` as:

```bash
cmake -DWITH_SYSTEMD=1 .  # for systemd
cmake -DWITH_UPSTART=1 .  # for upstart
cmake -DWITH_SYSVINIT=1 . # for sysv
```

### Installing

```bash
sudo make install
```

To check the installed files check `install_manifest.txt` on the project root.

### Running

```bash
 fancontrold -h
 Usage: ./fancontrold [OPTIONS]... [HWMON_NAME]
 Copyright (C) 2019 Analog Devices, Inc.
 This is free software; see the source for copying conditions.
 There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A
 PARTICULAR PURPOSE.

  -v, --verbose		Verbose.
  -s, --sleep		Sleep time between temperature checks.
  -h, --help		Print this help.
```