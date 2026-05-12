# Sensor Telemetry & Noise Filtering Engine

A high-performance C++ utility designed to bridge raw sensor hardware and autonomous navigation stacks. 

## Key Features
- **Data Ingestion:** Uses C++ Structs to parse raw telemetry strings into organized data packets.
- **Noise Reduction:** Implements a Moving Average Filter using a Queue-based sliding window.
- **Latency Optimization:** Leverages Pass-by-Reference to ensure zero-copy data processing.
- **Memory Safety:** Follows RAII principles with STL containers to ensure zero memory leaks.

## Project Structure
- `/src`: Core C++ implementation logic.
- `/include`: Header files for custom data structures.
- `/data`: Sample telemetry logs (JSON/CSV).

## Status
Active Development - Optimizing filter response for high-frequency Lidar data.
