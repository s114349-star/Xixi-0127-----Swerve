// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.chassis;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 馬達極限速度(12v),換成公尺/秒,功率100%(小一點較安全)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 每秒轉0.75圈,(Radians)弧度

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // 搖桿數值小於0.1無視,平移和旋轉
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // 開迴圈電壓,(Velocity)閉迴圈
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();//輪子內八
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);// 傳入MaxSpeed顯示速度條比例

    private final CommandXboxController xbox = new CommandXboxController(0);

    public final chassis drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(

                drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // x軸相反,搖桿比例(-1~1)*MaxSpeed
                        .withVelocityY(-xbox.getLeftX() * MaxSpeed)
                        .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // 旋轉相反
                ));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));// disable時

        xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));// 長按時輪子內八
        xbox.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))));// 長按時只轉向不移動

        xbox.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.resetPose(new Pose2d());
            drivetrain.seedFieldCentric();
        })); // 場地座標歸零

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
