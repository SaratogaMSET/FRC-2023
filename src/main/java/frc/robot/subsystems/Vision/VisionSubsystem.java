/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightResults;
import frc.robot.util.wrappers.VisionMeasurement;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Vision;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera ll2 = new PhotonCamera("OV5647");
    public PhotonPoseEstimator photonPoseEstimator;

   
    AprilTagFieldLayout field = null;  //TODO: fill in 

    private double[] distances = new double[]{
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1
    };

    private double[] getDistances() {
        distances = new double[]{
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1
        };

        for (var r : getTargets()) {
            var tmp = r.getBestCameraToTarget();
            distances[(int) r.getFiducialId() - 1] = Math.hypot(tmp.getX(), tmp.getZ()); //TODO: figure out what coordinate system photonvision uses
            // we only care about the horizontal distance not the vertical. 
        }

        return distances;
    }
    
    public VisionSubsystem(){
        // try {
        //     field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile); //why does this not work man
        // } catch (Exception e){
        //     SmartDashboard.putNumber("ITS JOEVER", 1);
        // }

        field = FieldConstants.aprilTags;
        photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP, ll2, Constants.Vision.robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }
    // Construct PhotonPoseEstimator
    public PhotonPipelineResult ll2_Result(){
        return ll2.getLatestResult();
    }

    public PhotonPipelineResult getLatestResults(){
        if (ll2_Result().hasTargets()){
            return ll2_Result();
        }
        return ll2_Result();
    }

    public boolean has_targets(){
        return ll2_Result().hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(){
        if (has_targets()) return ll2_Result().getTargets();
        return new ArrayList<PhotonTrackedTarget>();
    }

    /**
     * @return the best target acquired by the camera, can return Null
     */
    public PhotonTrackedTarget getBestTarget(){
        if (has_targets()) return getLatestResults().getBestTarget();
        return null;
        //returns null if there is no target*
    }

    public int getTagID(){
        try {
            return getBestTarget().getFiducialId();
        } catch (Exception e) {
            System.out.println("no targets");
            return -1;
        }
    }

    public Transform3d getCamTran(){
        try {
            return getBestTarget().getBestCameraToTarget();
        } catch (Exception e) {
            System.out.println("No Targets?");
            return new Transform3d();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }

    public double getLatency(){
        return ll2_Result().getLatencyMillis();
    }

    public VisionMeasurement getLatestMeasurement(){
        return new VisionMeasurement(
            has_targets(),
            getLatency(),
            getTagID(),
            photonPoseEstimator.update().get().estimatedPose.toPose2d(),
            new Pose2d(getCamTran().getTranslation().toTranslation2d(), getCamTran().getRotation().toRotation2d()),
            getDistances() 

        );
    }

    public Vector<N3> getScaledSTDDevs(){
        if(photonPoseEstimator.update().isPresent()){
            var estimation = photonPoseEstimator.update().get();
            double sumDistance = 0;
            for(var target: estimation.targetsUsed){
                var bestCamera = target.getBestCameraToTarget();
                sumDistance += Math.sqrt(Math.pow(bestCamera.getX(), 2) + Math.pow(bestCamera.getY(), 2) + Math.pow(bestCamera.getZ(), 2));
            }
            double targetsUsed = (double) estimation.targetsUsed.size();
            double averageDistance = sumDistance/ targetsUsed;
            return VecBuilder.fill((Vision.xyCoeff * averageDistance)/targetsUsed, (Vision.xyCoeff * averageDistance/targetsUsed), (Vision.rotationCoeff * averageDistance/targetsUsed));
        }
        else{
            return VecBuilder.fill(Vision.xyCoeff,Vision.xyCoeff,Vision.rotationCoeff);
        }
    }

    public double getTimestamp(){
        return Timer.getFPGATimestamp() - getLatency();
    }

    public int getPipelineIndex(){
        return ll2.getPipelineIndex();
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pipeline INdex", getPipelineIndex());
  }


}