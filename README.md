# Adaptive Gate Controller (Arduino)

An Arduino-based **automatic gate controller** with adaptive timing, safety features, and a learning mode.

The system controls gate opening and closing using a state machine and dynamically adjusts motor timing based on real travel measurements, improving smoothness and reliability over time.

## Features

- State machine control (IDLE, OPENING, CLOSING, LEARNING)
- Automatic learning of gate travel time
- Adaptive full-speed and slow-down phases
- Photocell, emergency stop, and optional overcurrent protection
- Intelligent recovery from mid-travel interruptions
- Flashing warning light support
- RF receiver input and I²C status interface
- Time-based position estimation

## Hardware

- Arduino-compatible board
- Motor driver (direction + PWM)
- Open/close limit switches
- Photocell and emergency stop
- Optional current sensor and I²C master

## Notes

Designed for real-world gates where speed and friction may change over time.

## License

MIT License
