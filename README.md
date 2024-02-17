# Bragi

####  Bragi VisionTable feature branch
Robovikes uses the Limelight Coprocessor and PhotonVision for robot computer vision. This repo is for testing and experimentation for simple Limelight tasks.

#### Limelight Standard Operating Procedure

Here is a standard operating procedure for getting a Limelight using photonvision working for the RoboVikes team.

### Background

The Limelight uses its configuration to identify which objects in its image is most likely to be the target. It then sends this information to the RoboRIO where the main code is running.The Limelight is a raspberry pi compute module with a camera

For information about how to install the Limelight software and access the dashboard, visit the [official Limelight documentation](https://docs.Limelightvision.io/en/latest/)


### Necessary Equipment

- Computer with Windows
- USB micro b  to USB port (v2 or 2+)
- USB type C to USB port (v3)
- Limelight v2 or v3 (https://limelightvision.io/collections/products)
- PhotonVision calibration sheet, calipers, with clipboard, and tape

### Determine Limelight version:

- Open the limelight by removing the 4x 1.5 mm start screws
- Remove lid
- Locate version info. The version should be v1, 2, 2+, or 3.

Robovikes has Limelight v2 and v3

### Install software

- Download and install Limelight usb drivers (https://limelightvision.io/pages/downloads)
- Download and install Balena Etcher (https://etcher.balena.io/)
- Download the latest PhotonVision firmware image release [here](https://github.com/PhotonVision/photonvision/releases) (e.g. `photonvision-v2024.1.4-linuxarm64-image_limelight2.xz`)
- Connect usb cable to Limelight and plug into computer. If it doesn't show up, make sure you have installed the Limelight USB drivers correctly
- Open BalenaEtcher
- Flash PhotonVision firmware
- Install firmware on Limelight 

- Install photonVision library

https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html

## V2, V2+

- Wire it up following these instructions https://docs.photonvision.org/en/latest/docs/installation/wiring.html
Network according to the diagram below: (https://docs.photonvision.org/en/latest/docs/installation/networking.html)
- Add passive 12 volt passive PoE ethernet to the coprocessor side or power it directly using +12V to the PDV/PDB or or a power bank on the micro USB (if allowed by this year’s rules).
- Connect the Limelight to a network (through radio, router or switch) that is shared by the laptop.
- Power on limelight
- Connect to RoboRio
- In browser connect to photon vision at the address `photonvision.local:5800`

- Set Team Number to `701`
- Import Settings
    - Hardware Config `hardwareConfig.json`
    - AprilTag Layout `AprilTagLayout`
- Set Static IP to `10.7.1.11` for first camera and `10.7.1.12` for second camera
- On Dashboard
    - Under Cameras, turn off Driver Mode
    - Click APRILTAG
        - Select AprilTag 36h11 (6.5in)
    - In the camera pipeline window, use the interface to update the camera name and pipeline.
        - Note: the camera name must match the
        - Rename Camera to photonvision (default is Camera_Module_v1 and each camera name must be unique?)
        - Rename Pipeline to AprilTag2024
        - Select Type AprilTag

 

## Calibration

- Print the PhotonVision [chessboard 8x8x1 calibration sheet](`src\main\java\frc\robot\utils\limelight\calibrationTarget-Chessboard.pdf`)
- Measure the square with calipers - should be close to 1". Write actual value (e.g. 0.97") in.
- Tape the sheet to a clipboard or piece of wood with a large border.
- Follow calibration instructions.
- Go to Cameras tab
- Start calibration.                              
- Move board around until colored lines appear and collect an image. Repeat at least 12 times.
- save calibration.
- repeat calibration for other resolutions. The higher the resolution, the lower frame rate.

## Port Forwarding

- In the `Robot.java` code, add the following line to the robotInit() function.

`Add PortForwarder.add(5800, "photonvision.local", 5800);`

## Camera Streams (For dashboard?):
Cameras will stream on ports `1181` and `1182` for the first camera and `1183` and `1184` for the second camera.

- Once connected and ready to connect to the RoboRIO, set a static IP to `10.7.1.11` for the first Limelight and `10.7.1.12` for the second Limelight.

## Observing results
- Update `VisionSubsystem.java` with any Limelight related variables you want to observe
- Connect Limelight to RoboRio
- Connect to RoboRio using driver station
- Open ShuffleBoard
- Point robot to AprilTags and watch the ShuffleBoard update 


## Continuous integration (CI)

CI workflows are run prior to pushing or merging code in github to improve and enforce code standards and quality to downstream users.

- `CI-build.yaml`
    - This workflow tries to build the robot code prior to accepting the code in the remote github repository.
    - The build uses the `settings.gradle` options.
    - If the build fails, then the code is not accepted into the remote github repository.
    - To pass a failing build, fix the code until the build is successful locally, then commit, merge and push your changes.

- `CI-format-checker.yaml`
    - This workflow checks the format of the code format for consistency which improves code readability.
    - The build uses `spotless` settings in the `build.gradle` file. Among other settings, the `googleJavaFormat` is used for checking the formatting of `java` code. 
    - Make sure the `.gitattributes` file is present if code is being developed or committed to the remote branch from different OSs (PC and Mac[Mr. Murphy]).
    - If the build fails, then the code is not accepted into the remote github repository.
    - To pass failing code, run the `.\gradlew spotlessApply` command fix the code formatting. Run the `.\gradlew spotlessCheck` command to until the build is successful locally, then commit, merge and push your changes.
    - TL;DR: Run `.\gradlew spotlessApply` in the terminal in the `.\BragiCode` directory prior to pushing code to the remote.
