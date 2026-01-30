// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.awt.Robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;

public class vision extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_PoseEstimator = new DifferentialDrivePoseEstimator(null, null, 0, 0,
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));// 位置估計器

  public vision() {// 處理視覺系統
    
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation("limelight", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = false;
    if (mt2.tagCount == 0) {// 沒有真偵測到AprilTag 拒絕更新
      doRejectUpdate = true;
    } else if (doRejectUpdate == false) {
      m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));// 不信任mt2的陀螺儀(信任值越小越信任),信任X,Y座標
      m_PoseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }


  }

}
