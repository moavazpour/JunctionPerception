# JunctionPerception

Camera-only 3D perception for road intersections, built for edge deployment.

- Monocular 3D detection of road users: car, bus, truck, trailer, pedestrian, cyclist, e-scooter rider
- Ground and road-marking understanding with automatic camera calibration
- 3D tracking with speed, acceleration, lane assignment and time-to-stop-line
- Near-miss detection using surrogate safety measures (TTC, PET, DRAC)
- Self-monitoring of camera shake, calibration drift, weather and visibility, and occlusion
- Privacy by design: frames are processed in memory only and the system emits metadata
- Standard outputs (MQTT, ETSI CPM, virtual detectors) and a Three.js live 3D web panel

Core in C++20 with CUDA/TensorRT. Model training in Python/PyTorch. Ships as Docker images.

See [docs/PLAN.md](docs/PLAN.md) for the roadmap (in Persian).
