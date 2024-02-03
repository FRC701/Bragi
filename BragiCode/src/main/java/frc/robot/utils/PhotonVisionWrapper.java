// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.kVision;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/** Add your docs here. */
public class PhotonVisionWrapper {
  static HashMap<String, PhotonCamera> cams = new HashMap<String, PhotonCamera>();
  PhotonCamera cam;

  public PhotonVisionWrapper(String name) {
    if (!cams.containsKey(name)) {
      cams.put(name, new PhotonCamera(name));
    }
    cam = cams.get(name);
  }

  public double getYaw() {
    var results = cam.getLatestResult();
    if (results == null) {
      return 0;
    }
    return (results.hasTargets()) ? results.getBestTarget().getYaw() : 0;
  }

  public double getDistance() {
    var results = cam.getLatestResult();
    double distance = -1;

    SmartDashboard.putBoolean("hasResults", results != null);
    if (results == null) {
      return -1;
    }
    SmartDashboard.putBoolean("hasTargets", results.hasTargets());
    if (results.hasTargets()) {
      double pitch = results.getBestTarget().getPitch();
      distance =
          PhotonUtils.calculateDistanceToTargetMeters(
              kVision.CAMERA_HEIGHT,
              kVision.TARGET_HEIGHT,
              kVision.CAMERA_PITCH,
              Units.degreesToRadians(pitch));
      SmartDashboard.putNumber("distanceToTarget", distance);
      return distance;
    }
    return -1;
  }

  public boolean hasTargets() {
    return cam.getLatestResult().hasTargets();
  }

  public double getTimestamp() {
    return Timer.getFPGATimestamp() - (cam.getLatestResult().getLatencyMillis() / 1000);
  }
}
