package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class chassis extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // (Loop)週期 4/ms
    private Notifier m_simNotifier = null;// 通知器
    private double m_lastSimTime;// 紀錄上次執行的時間

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;// 視角旋轉,多以藍色聯盟為基準

    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private boolean m_hasAppliedOperatorPerspective = false;// 是否套用聯盟視角

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();// 平移
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();// 轉向
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();// 旋轉

    private final SwerveRequest.ApplyRobotSpeeds speedsApplier = new SwerveRequest.ApplyRobotSpeeds();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            // Translation(平移),Rotation(旋轉),Elevator(升降),Arm(機械臂)
            new SysIdRoutine.Config(
                    null, // 電壓上升速度:null= 默認值1v/秒
                    Volts.of(4), // 最大電壓4v (安全
                    null, // 測試最大時長:null= 默認10秒
                    //
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString()) // 紀錄sysld 狀態
            ),
            new SysIdRoutine.Mechanism( // 控制
                    output -> setControl(m_translationCharacterization.withVolts(output)), // 控制電壓
                    null, // 額外數據紀錄(編碼器位置)
                    this)); // 要用在現在這個子系統
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second), // Math.PI=圓周率
                    Volts.of(Math.PI),
                    null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public chassis(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);// 呼叫父類別
        if (Utils.isSimulation()) { // 若使用模擬器執行
            startSimThread(); // kSimLoopPeriod和Notifier
        }
        configurePPAuto();
    }

    public chassis(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency, // 里程計更新頻率
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePPAuto();
    }

    public chassis(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, // 3x1矩形,機器人在(x,y,0)(座標與角度),數值越小越信任編碼器
            Matrix<N3, N1> visionStandardDeviation, // 視覺系統偏差
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    private void configurePPAuto() {
        try {
            AutoBuilder.configure(
                    () -> getState().Pose,
                    (pose) -> resetPose(pose),
                    () -> getState().Speeds,
                    (speeds, feedForwards) -> setControl(
                            speedsApplier.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedForwards.robotRelativeForcesX())
                                    .withWheelForceFeedforwardsY(feedForwards.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(0, 0, 0),
                            new PIDConstants(0, 0, 0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception error) {
            DriverStation.reportError("Error occured when configuring AutoBuilder: ", error.getStackTrace());
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
