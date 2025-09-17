# Vision System for Robot Control

This package provides object detection and robot control capabilities using YOLOv8 and ROS 2.

## Features

- Real-time object detection with YOLOv8
- Optimized for edge devices (Raspberry Pi 4)
- ROS 2 integration for robot control
- Support for various optimization techniques:
  - Model quantization (FP16/INT8)
  - Layer freezing
  - Mosaic augmentation
  - TensorRT acceleration (optional)

## Installation

### Prerequisites
- ROS 2 Humble
- Python 3.8+
- PyTorch
- OpenCV
- Ultralytics YOLO

### Setup

1. Clone the repository:
   ```bash
   cd ~/Dojo/src
   git clone <repository-url>
   ```

2. Install Python dependencies:
   ```bash
   pip3 install -r requirements.txt
   ```

3. Build the package:
   ```bash
   cd ~/Dojo
   colcon build --packages-select vision_system
   source install/setup.bash
   ```

## Usage

### Launch the Vision System
```bash
ros2 launch vision_system vision_system.launch.py
```

### Parameters

Edit `config/object_detector_params.yaml` to configure:
- Model settings
- Detection thresholds
- Control parameters
- Optimization flags

### Topics

#### Subscribed
- `/camera/image_raw` - Input camera feed

#### Published
- `/cmd_vel` - Velocity commands for robot control
- `/detections` - Detection results

## Optimization Techniques

### 1. Model Optimization
- **Quantization**: Reduces model size and improves inference speed
- **Layer Freezing**: Freezes early layers during initial training
- **Pruning**: Removes unnecessary model weights

### 2. Image Processing
- Resolution scaling
- Color space optimization
- Batch processing

### 3. Hardware Acceleration
- TensorRT support
- GPU acceleration (if available)
- Multi-threaded processing

## Customization

### Training Custom Models
1. Prepare your dataset in YOLO format
2. Configure training parameters in `config/training.yaml`
3. Run training script:
   ```bash
   python3 train.py
   ```

### Adding New Features
1. Create a new Python module in `vision_system/`
2. Add your node to `setup.py`
3. Create a launch file if needed

## Troubleshooting

### Common Issues
1. **Low FPS**
   - Reduce input resolution
   - Enable model quantization
   - Use a smaller YOLO model

2. **High Memory Usage**
   - Reduce batch size
   - Disable visualization if not needed
   - Use FP16 or INT8 quantization

3. **Detection Performance**
   - Adjust confidence threshold
   - Train with more diverse data
   - Fine-tune NMS parameters

## License

Apache 2.0

## Contributing

Contributions are welcome! Please submit a pull request or open an issue.
