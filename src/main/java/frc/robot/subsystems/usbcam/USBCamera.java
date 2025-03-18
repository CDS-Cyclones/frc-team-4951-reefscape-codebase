// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.usbcam;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class USBCamera extends SubsystemBase {
  UsbCamera cam;
  MjpegServer server, server2;
  CvSink cvSink;
  CvSource cvSource;

  /** Creates a new USBCameraSubsystem. */
  public USBCamera(String camName, int camID) {
    cam = new UsbCamera(camName, camID);
    server = new MjpegServer("server_"+camName, 1181);
    server.setSource(cam);
    cvSink = new CvSink("opencv_" + camName);
    cvSink.setSource(cam);
    cvSource = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    server2 = new MjpegServer("serve_Blur", 1182);
    server2.setSource(cvSource);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
