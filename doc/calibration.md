# Camera and Sensor Calibration

SVO supports several camera models. This document describes how to calibrate cameras using common tools and convert the output to SVO's YAML format.

## Camera Models

### Pinhole + Radial-Tangential

This is the standard distortion model used in OpenCV (also known as `plumb_bob`). Calibrate using any OpenCV-based calibration tool. The output format is:

```
camera matrix
fx 0 cx
0 fy cy
0 0 1

distortion
d0 d1 d2 d3 0
```

For use with SVO, fill in the following YAML template:

```yaml
cameras:
- camera:
    distortion:
      parameters:
        cols: 1
        rows: 4
        data: [$d0, $d1, $d2, $d3]
      type: radial-tangential
    image_height: $image_height
    image_width: $image_width
    intrinsics:
      cols: 1
      rows: 4
      data: [$fx, $fy, $cx, $cy]
    label: cam0
    line-delay-nanoseconds: 0
    type: pinhole
  T_B_C:
    cols: 4
    rows: 4
    data: [ 1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.]
  serial_no: 0
  calib_date: 0
  description: '$camera_name'
label: $camera_name
```

`T_B_C` is the pose of the camera frame in the IMU/body frame. This is used when SVO is configured to use the IMU.

### Pinhole + Equidistant

This is a generic distortion model that can handle very different fields of view ([paper](http://www.ee.oulu.fi/mvg/files/pdf/pdf_697.pdf)), making it suitable for both pinhole and fisheye cameras. OpenCV 3.0+ [supports this model](http://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html).

To calibrate, use [Kalibr](https://github.com/ethz-asl/kalibr). Refer to the [Kalibr calibration manual](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) for details.

Convert the Kalibr output to SVO format using:

```sh
python svo_ros/scripts/kalibr_to_svo.py --kalibr <output_of_kalibr>
```

### Omnidirectional

This model combines projection and distortion into a single representation. It works for fisheye and catadioptric cameras. Calibrate using the [OCamCalib Matlab Toolbox](https://sites.google.com/site/scarabotix/ocamcalib-toolbox).

Convert the Matlab output to SVO format using:

```sh
python svo_ros/scripts/omni_matlab_to_rpg.py <matlab_output>
```

## Visual-Inertial Calibration

For calibrating visual-inertial sensor systems (camera-IMU extrinsics and time offsets), we recommend [Kalibr](https://github.com/ethz-asl/kalibr).

See [frontend_fla.md](./frontend/frontend_fla.md) for the format of calibration files for stereo and stereo + IMU configurations, including:
- Camera intrinsics and extrinsics (`T_B_C`)
- IMU noise parameters (`sigma_omega_c`, `sigma_acc_c`, `sigma_omega_bias_c`, `sigma_acc_bias_c`)
- IMU initialization values (velocity, biases)

## Example Calibration Files

Example calibration files are provided under `svo_ros/param/calib/`:

| File | Description |
|---|---|
| `svo_test_pinhole.yaml` | Test pinhole camera |
| `euroc_mono.yaml` | EuRoC monocular |
| `euroc_stereo.yaml` | EuRoC stereo |
| `bluefox_25000826_fisheye.yaml` | Bluefox fisheye camera |
| `fla_stereo_imu.yaml` | FLA stereo + IMU |
| `davis_flyingroom.yaml` | DAVIS event camera |

## Tips

* Always use a **global shutter** camera. A good choice is [the Bluefox camera](https://www.matrix-vision.com/USB2.0-single-board-camera-mvbluefox-mlc.html) from MatrixVision.
* The translation components of the camera-IMU extrinsics are sometimes poorly calibrated (likely due to lack of accelerometer excitation). Double-check your calibration results before use.
* The `kalibr_to_svo.py` script also handles multi-camera setups with IMU.
