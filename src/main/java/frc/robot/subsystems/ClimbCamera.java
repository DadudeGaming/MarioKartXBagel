package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;

import java.security.PublicKey;

import edu.wpi.first.cameraserver.CameraServer;

public class ClimbCamera {
    public UsbCamera climbCamera;

    public ClimbCamera() {  }

    public void startCamera() {
        // // start capture from the camera at the front plugged into USB slot 0
        climbCamera = CameraServer.startAutomaticCapture();
        // set the stream's resolution to 320x240
        climbCamera.setResolution(320, 240);
        // set the stream's frames per second to 15
        climbCamera.setFPS(15);
        climbCamera.setVideoMode(PixelFormat.kGray, 320, 240, 15);
    }
}
