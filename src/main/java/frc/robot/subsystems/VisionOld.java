// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.security.PublicKey;
// import java.util.List;
// import java.util.Optional;


// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.counter.UpDownCounter;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.Robot;
// import swervelib.SwerveDrive;

// public class Vision extends SubsystemBase {
//   /** Creates a new ExampleSubsystem. */
  
//   // Change this to match the name of your camera (please do this)
  
//   private PhotonCamera frontCamera;

//   private PhotonPoseEstimator photonEstimator;

//   private Matrix<N3, N1> curStdDevs;

//    /**
//      * Latency alert to use when high latency is detected.
//      */
//     public static Alert latencyAlert;
    
//       // Determines if there are any targets in view
//       private boolean hasTargets;
    
//       public Vision() {
//         frontCamera = new PhotonCamera("PhotonFront");
    
//         photonEstimator =
//                     new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
//             photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//       }
    
    
//       public void updatePoseEstimation(SwerveDrive swerveDrive) {
//         for (Cameras camera : Cameras.values()) {
//           Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
//           if (poseEst.isPresent()) {
//             var pose = poseEst.get();
//             swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
//                                              pose.timestampSeconds,
//                                              camera.curStdDevs);
//           }
//         }
//       }
    
//       public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
//         Optional<EstimatedRobotPose> visionEst = Optional.empty();
    
//         for (var change : frontCamera.getAllUnreadResults()){
//           visionEst = photonEstimator.update(change);
//           updateEstimationStdDevs(visionEst, change.getTargets());
//         }
    
//         return visionEst;
    
//       }
    
    
//        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
//             if (estimatedPose.isEmpty()) {
//                 // No pose input. Default to single-tag std devs
//                 curStdDevs = VisionConstants.kSingleTagStdDevs;
    
//             } else {
//                 // Pose present. Start running Heuristic
//                 var estStdDevs = VisionConstants.kSingleTagStdDevs;
//                 int numTags = 0;
//                 double avgDist = 0;
    
//                 // Precalculation - see how many tags we found, and calculate an average-distance metric
//                 for (var tgt : targets) {
//                     var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//                     if (tagPose.isEmpty()) continue;
//                     numTags++;
//                     avgDist +=
//                             tagPose
//                                     .get()
//                                     .toPose2d()
//                                     .getTranslation()
//                                     .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
//                 }
    
//                 if (numTags == 0) {
//                     // No tags visible. Default to single-tag std devs
//                     curStdDevs = VisionConstants.kSingleTagStdDevs;
//                 } else {
//                     // One or more tags visible, run the full heuristic.
//                     avgDist /= numTags;
//                     // Decrease std devs if multiple targets are visible
//                     if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
//                     // Increase std devs based on (average) distance
//                     if (numTags == 1 && avgDist > 4)
//                         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//                     else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
//                     curStdDevs = estStdDevs;
//                 }
//             }
//         }
    
//         /**
//          * Returns the latest standard deviations of the estimated pose from {@link
//          * #getEstimatedGlobalPose()}, for use with {@link
//          * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
//          * only be used when there are targets visible.
//          */
//         public Matrix<N3, N1> getEstimationStdDevs() {
//           return curStdDevs;
//       }
    
      
    
      
    
//       @Override
//       public void periodic() {
//         // This method will be called once per scheduler run
//       }
    
//       @Override
//       public void simulationPeriodic() {
//         // This method will be called once per scheduler run during simulation
//       }
    
       
    
    
    
//       /**
//        * Camera Enum to select each camera
//        */
//       enum Cameras
//       {
//         /**
//          * Left Camera
//          */
//         LEFT_CAM("left",
//                  new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
//                  new Translation3d(Units.inchesToMeters(12.056),
//                                    Units.inchesToMeters(10.981),
//                                    Units.inchesToMeters(8.44)),
//                  VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
//         /**
//          * Right Camera
//          */
//         RIGHT_CAM("right",
//                   new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
//                   new Translation3d(Units.inchesToMeters(12.056),
//                                     Units.inchesToMeters(-10.981),
//                                     Units.inchesToMeters(8.44)),
//                   VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
//         /**
//          * Center Camera
//          */
//         CENTER_CAM("center",
//                    new Rotation3d(0, Units.degreesToRadians(18), 0),
//                    new Translation3d(Units.inchesToMeters(-4.628),
//                                      Units.inchesToMeters(-10.687),
//                                      Units.inchesToMeters(16.129)),
//                    VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));
    
//       Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
//                    Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
//              latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
  
//          camera = new PhotonCamera(name);
        
//          // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
//          robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
   
//          photonEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
//                                                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                                                  robotToCamTransform);
//          photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
   
//          this.singleTagStdDevs = singleTagStdDevs;
//          this.multiTagStdDevs = multiTagStdDevsMatrix;
//        }
// }
