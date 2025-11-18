# Flight-Exec

A modular, safety-focused flight execution system for ArduPilot-compatible drones with comprehensive monitoring, logging, and safety features.

## Features

- **Modular Architecture**: Clean separation of services, utilities, and main entry points
- **Comprehensive Safety Monitoring**: Battery, heartbeat, Jetson system stats, and FC telemetry
- **Safe Landing Procedures**: Multiple fallback methods (RTL → LAND → simple_goto → disarm)
- **Detailed Logging**: All operations logged to timestamped files and console simultaneously
- **Configuration Management**: Centralized config file for all flight parameters
- **Three Main Operations**:
  - **Flight**: Takeoff, hold position, land with full safety monitoring
  - **Arming**: Ground testing arm/disarm operations
  - **Mission**: Point-to-point waypoint navigation

## Project Structure

```
Flight-Exec/
├── config.py                 # Centralized configuration
├── main/                      # Main entry points
│   ├── flight_main.py        # Flight operations
│   ├── arming_main.py         # Arming service
│   └── mission_main.py        # Mission service
├── services/                  # Flight operation services
│   ├── arming_service.py      # Arm/disarm operations
│   ├── battery_monitor.py     # Battery monitoring
│   ├── enhanced_takeoff_service.py  # Takeoff/landing
│   ├── fc_telemetry_monitor.py     # FC telemetry
│   ├── heartbeat_monitor.py   # Connection monitoring
│   ├── jetson_monitor.py      # System monitoring
│   ├── mission_service.py     # Waypoint navigation
│   ├── position_hold_service.py    # Position holding
│   └── safety_manager.py     # Safety management
├── utils/                     # Utility modules
│   ├── detect_controller.py  # FC detection/connection
│   ├── geometry_utils.py      # Distance calculations
│   ├── logging_utils.py       # Logging setup
│   ├── port_utils.py          # Serial port management
│   ├── shell_utils.py         # Shell command execution
│   └── system_stats.py        # System statistics
└── logs/                      # Log files directory (created automatically)
```

## Installation

### Prerequisites

- Python 3.8+
- ArduPilot-compatible flight controller
- Linux system (tested on Jetson platforms)
- Serial port access to flight controller

### Dependencies

Install required packages:

```bash
pip install -r requirements.txt
```

Required packages:
- `dronekit` - MAVLink communication
- `pyserial` - Serial port access
- `psutil` - System monitoring (optional, for Jetson stats)

## Configuration

All configuration is managed through `config.py`. Three configuration classes are available:

### FlightConfig
- `target_altitude`: Target altitude for flight (default: 5.0m)
- `hold_seconds`: Time to hold position (default: 10.0s)
- `enable_battery_monitor`: Enable battery monitoring (default: True)
- `enable_heartbeat_monitor`: Enable heartbeat monitoring (default: True)
- `enable_jetson_monitor`: Enable Jetson system monitoring (default: True)
- `enable_fc_telemetry_monitor`: Enable FC telemetry monitoring (default: True)
- `logging_detailed`: Use DEBUG level logging (default: True)
- `logging_dir`: Log directory (default: "logs")

### ArmingConfig
- `hold_seconds`: Time to hold armed state (default: 5.0s)
- `disable_checks`: Disable arming checks for ground testing (default: True)
- All monitoring toggles and logging settings

### MissionConfig
- `hover_delay`: Time to loiter at waypoint (default: 3.0s)
- `gps_wait_seconds`: GPS fix wait time (default: 10s)
- `groundspeed`: Navigation speed (default: 0.1 m/s)
- All monitoring toggles and logging settings

To customize, edit `config.py` or use CLI arguments to override.

## Usage

### Flight Operations

Basic flight: takeoff, hold position, land.

```bash
python main/flight_main.py
# or
python -m main.flight_main
```

This will:
1. Connect to flight controller
2. Start all monitoring threads
3. Arm and takeoff to configured altitude
4. Hold position for configured time
5. Land and disarm

### Arming Service

Ground testing arm/disarm operations (no GPS required).

```bash
# Basic usage (uses config defaults)
python main/arming_main.py

# Custom hold time
python main/arming_main.py --hold-seconds 10

# Keep arming checks enabled
python main/arming_main.py --keep-checks

# Disable safety monitoring (not recommended)
python main/arming_main.py --no-safety-monitoring
```

**Arguments:**
- `--hold-seconds`: Seconds to hold armed state (default: from config)
- `--keep-checks`: Keep arming checks enabled
- `--no-safety-monitoring`: Disable safety monitoring threads

### Mission Service

Point-to-point waypoint navigation.

```bash
# Basic usage
python main/mission_main.py 5.0 10.0 5.0 5.0

# With custom hover delay and GPS wait
python main/mission_main.py 5.0 10.0 5.0 5.0 --hover-delay 5.0 --gps-wait 15

# Disable safety monitoring (NOT RECOMMENDED)
python main/mission_main.py 5.0 10.0 5.0 5.0 --no-safety-monitoring
```

**Arguments:**
- `takeoff_altitude`: Altitude to reach on takeoff (meters)
- `north_offset`: Meters to travel north from takeoff point
- `east_offset`: Meters to travel east from takeoff point
- `target_altitude`: Cruise altitude at waypoint (meters)
- `--hover-delay`: Seconds to loiter at waypoint (default: from config)
- `--gps-wait`: Seconds to wait for GPS fix (default: from config)
- `--no-safety-monitoring`: Disable safety monitoring threads

**Example:**
```bash
# Takeoff to 5m, fly 10m north and 5m east at 5m altitude, hover 5s, then land
python main/mission_main.py 5.0 10.0 5.0 5.0 --hover-delay 5.0
```

## Safety Features

### Monitoring Services

All three main operations include comprehensive monitoring:

1. **Battery Monitor**
   - Monitors voltage, current, and battery level
   - Triggers safe landing on critical battery (< 10% by default)
   - Warns on low battery (< 20% by default)

2. **Heartbeat Monitor**
   - Monitors connection status
   - Warns on delayed heartbeats (> 5s by default)
   - Triggers safe landing on connection loss (> 15s by default)

3. **Jetson Monitor**
   - Monitors CPU, memory, disk usage
   - Monitors system temperatures
   - Monitors GPU stats (if available)
   - Warns on high temperatures (> 80°C)

4. **FC Telemetry Monitor**
   - Monitors GPS status (fix type, satellites)
   - Monitors attitude (roll, pitch, yaw)
   - Monitors velocity and heading
   - Monitors EKF status (triggers safe landing on failure)
   - Monitors system status

### Safe Landing

Multiple fallback methods ensure safe landing:

1. **RTL (Return to Launch)**: Most reliable if home position is set
2. **LAND Mode**: Direct landing at current position
3. **simple_goto to Home**: Navigate to home then land
4. **Emergency Disarm**: Last resort if very low altitude

Safe landing is triggered on:
- Critical battery levels
- Connection loss
- EKF failure
- GPS fix loss during critical operations
- Manual interrupt (Ctrl+C)
- Any unhandled exceptions

## Logging

All operations log to both file and console:

- **File Logging**: Detailed logs saved to `logs/` directory
  - `flight_YYYY-MM-DD_HH-MM-SS.log`
  - `arming_YYYY-MM-DD_HH-MM-SS.log`
  - `mission_YYYY-MM-DD_HH-MM-SS.log`
- **Console Logging**: INFO+ level messages printed to console
- **Log Levels**: DEBUG (file only), INFO (file + console), WARNING, ERROR, CRITICAL

Logs include:
- All monitoring data (battery, heartbeat, system stats, telemetry)
- Flight operations (takeoff, hold, landing)
- Safety triggers and responses
- Connection status
- Errors and exceptions

## Architecture

### Modular Design

- **Services**: High-level flight operations (takeoff, landing, navigation)
- **Utils**: Low-level utilities (connection, logging, geometry)
- **Main**: Entry points that orchestrate services
- **Config**: Centralized configuration management

### Safety Manager

Central safety management system:
- Thread-safe safe landing triggers
- Home position management
- Safety state tracking
- Multiple landing fallback methods

### Monitoring Threads

All monitors run as daemon threads:
- Non-blocking operation
- Automatic cleanup on exit
- Thread-safe communication via events

## Development

### Adding New Services

1. Create service in `services/`
2. Import safety manager for safety features
3. Use config for parameters
4. Use logging utils for logging
5. Create main entry point in `main/` if needed

### Testing

For ground testing without GPS:
```bash
python main/arming_main.py --keep-checks
```

This uses STABILIZE mode and disables arming checks.

## Troubleshooting

### Connection Issues

- Check serial port permissions: `ls -l /dev/ttyACM*`
- Ensure no other processes are using the port
- Try different baud rates (115200, 57600, 921600)

### GPS Issues

- Wait for GPS fix before takeoff
- Check GPS antenna connection
- Verify GPS module is powered

### Safety Triggers

If safe landing triggers unexpectedly:
- Check battery levels
- Verify connection stability
- Check EKF status
- Review logs for details

## License

See LICENSE file for details.

## Contributing

1. Follow modular architecture principles
2. Integrate with safety manager for safety features
3. Use config for all parameters
4. Add comprehensive logging
5. Test thoroughly before flight

## Safety Disclaimer

**WARNING**: This software controls aircraft. Always:
- Test in safe, open areas
- Have emergency procedures ready
- Verify all safety systems before flight
- Follow local regulations
- Never disable safety monitoring in flight
- Review logs after each flight

Use at your own risk. The authors are not responsible for any damage or injury.
