// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.leds.leds;
import frc.robot.subsystems.superstructure.leds.ledsConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LocalADStarAK;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase 
implements Vision.VisionConsumer
// implements TestVision.EstimateConsumer
{
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

                private final Timer lastPoseTimer = new Timer();

    RobotConfig config = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(TunerConstants.FrontLeft.WheelRadius, TunerConstants.FrontLeft.DriveInertia, WHEEL_COF, DCMotor.getKrakenX60(1), 60, 2),
        DriveConstants.TRACK_WIDTH.in(Meters)
    );
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        config,
        Units.rotationsToRadians(DriveConstants.MAX_ANGULAR_VELOCITY)
    );
        // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    WHEEL_COF));

    private Pose2d targetPose;
    private SwerveSetpoint previousSetpoint;
 private PIDController translationalController = new PIDController(
        0.01, 0, 0);
        // 5.25, 0, 0.3); 
    private PIDController rotationalController = new PIDController(
        0.01, 0, 0);
private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    private double robotAngle;
   // private leds led;
    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(0, 0.0, 0.0), new PIDConstants(0, 0.0, 0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        //            if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
        //      robotAngle =  Math.atan2(getPose().getY() - FieldConstants.Hub.topCenterPoint.getY(), (getPose().getX() - FieldConstants.Hub.topCenterPoint.getX()));
        // } else {
        //      robotAngle =  Math.atan2(getPose().getY() - AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.toTranslation2d().getY()), (getPose().getX() - AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX())));
        // }

        //led = new leds();
    }

    double robotDistance = 0;
    @Override
    public void periodic() {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
             robotAngle =  Math.atan2(getPose().getY() - FieldConstants.Hub.topCenterPoint.getY(), (getPose().getX() - FieldConstants.Hub.topCenterPoint.getX())) + Math.PI;
        } else {
             robotAngle =  Math.atan2(getPose().getY() - AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.toTranslation2d().getY()), (getPose().getX() - AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX()))) + Math.PI;
        }
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Logger.recordOutput("Travel rotation", travelRotation.getDegrees());

        Logger.recordOutput("isAligned", isAligned());

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
        //RobotState.getInstance().addOdometryPose( new OdometryObservation(Timer.getTimestamp(), getModulePositions(), Optional.ofNullable(gyroInputs.connected ? rawGyroRotation : null)) );

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && frc.robot.constants.Constants.currentMode != frc.robot.constants.Constants.Mode.SIM);
            
        //         if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
        //     robotDistance = getPose().getTranslation().getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        // } else{
        //     robotDistance = getPose().getTranslation().getDistance(AllianceFlipUtil.apply((FieldConstants.Hub.topCenterPoint.toTranslation2d())));        
        // }

        Logger.recordOutput("ShotEstimator/Distance", robotDistance);
        Logger.recordOutput("RobotAngle", robotAngle);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.01);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }
    public void setMinimumBumpVelocity(){
        if( getPose().getX() > 3 && getPose().getX() < 5){
            if(getPose().getY() > 1.5 && getPose().getY() < 3.5 || getPose().getY() > 4.5 && getPose().getY() < 6.5){
                if(Math.hypot(getChassisSpeeds().vxMetersPerSecond,getChassisSpeeds().vyMetersPerSecond)< 2){
                    runVelocity(new ChassisSpeeds(getRotation().getSin()*2,getRotation().getCos()*2,getRotation().getDegrees()));
                }
        }}
    }

    public Command setMinimumBumpVelocityCommand(){
        return Commands.runOnce(()->setMinimumBumpVelocity());
    }
    /** Returns the current odometry pose. */
      @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return new Pose2d(poseEstimator.getEstimatedPosition().getMeasureX(), poseEstimator.getEstimatedPosition().getMeasureY(), poseEstimator.getEstimatedPosition().getRotation());
  }
    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }


    /** Adds a new timestamped vision measurement. */
     @Override
     public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
         poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
     }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
public void setRotationX () {
        modules[0].setRotation(new Rotation2d(0.787));
        modules[1].setRotation(new Rotation2d(2.355));
        modules[2].setRotation(new Rotation2d(0.787));
        modules[3].setRotation(new Rotation2d(2.355));
    }

public Command setRotationXCommand () {
    return Commands.runOnce(() -> setRotationX());
}

public void driveRobotRelative(ChassisSpeeds speeds){
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    runVelocity(previousSetpoint.robotRelativeSpeeds());
}

public Command driveToPoseAuto(Supplier<Pose2d> targetPose){
    return Commands.run(() -> driveToPose(() -> new ChassisSpeeds(0,0,0), targetPose));
}





public void driveToPose(Supplier<ChassisSpeeds> targetSpeeds, Supplier<Pose2d> targetPose) {
    this.targetPose = targetPose.get();
    double targetRotation = targetSpeeds.get().omegaRadiansPerSecond;

    if(targetSpeeds.get().omegaRadiansPerSecond == 0){
        targetRotation = rotationalController.calculate(getPose().getRotation().getRadians(), this.targetPose.getRotation().getRadians());
    }

    boolean isNotDriving = targetSpeeds.get().vxMetersPerSecond == 0 && targetSpeeds.get().vyMetersPerSecond == 0;
    if(isNotDriving) {
        double targetTranlslationX = translationalController.calculate(0, Math.abs(getPose().getX() - this.targetPose.getX()));
        double targetTranlslationY = translationalController.calculate(0, Math.abs(getPose().getY() - this.targetPose.getY()));
        double distance = translationalController.calculate(0, getPose().getTranslation().getDistance(targetPose.get().getTranslation()));
        Translation2d targetTranlslation = new Translation2d(targetTranlslationX, targetTranlslationY);
        // ChassisSpeeds speeds = new ChassisSpeeds(isAligned() ? 0 : targetTranlslationX,
        //      isAligned() ? 0 : targetTranlslationY,
        //      isAligned() ? 0 : targetRotation);
        // ChassisSpeeds speeds = new ChassisSpeeds(isAligned() ? 0 : distance,
        //      0,
        //      isAligned() ? 0 : targetRotation);

        ChassisSpeeds speeds = new ChassisSpeeds(0,
             0,
             isAligned() ? 0 : targetRotation*0.25);

        Rotation2d travelRotation = this.targetPose.getTranslation().minus(getPose().getTranslation()).getAngle();
           System.out.println("travelRotation:" + travelRotation);
           System.out.println("targetPose:" + targetPose);
        this.runVelocity(speeds);
        // driveRobotRelative(speeds);


    }
        else {
            ChassisSpeeds speeds = new ChassisSpeeds(targetSpeeds.get().vxMetersPerSecond, targetSpeeds.get().vyMetersPerSecond, targetRotation);
            this.runVelocity(speeds);
            // driveRobotRelative(speeds);
        }

    
}

private ProfiledPIDController angleController = new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 4));
public void autoAlign(Supplier<Rotation2d> targetRotation){
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    double rotationSpeed = angleController.calculate(getPose().getRotation().getRadians(), targetRotation.get().getRadians());
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));

}

public boolean isAlignedToHub(){
    return getRotation().getRadians() > robotAngle -1 && getRotation().getRadians() < robotAngle +1;
}

public Command autoAlignToHub(){
    // if (isAlignedToHub()){
    //     led.setAllLedsCommand(ledsConstants.GREEN);
    // }
    // else{
    //     led.setAllLedsCommand(ledsConstants.RED);
    // }
    return Commands.run(() -> autoAlign(() -> new Rotation2d(robotAngle)));
    
}

public Command driveToPoseTeleop(Supplier<ChassisSpeeds> targetSpeeds, Supplier<Pose2d> targetPose){
    return Commands.run(() -> driveToPose(targetSpeeds, targetPose)).finallyDo(() -> this.targetPose = null);
}

public boolean atTargetPose() {
    if (targetPose == null) {
        return false;
    }
    //I need to make these constants
    double translationTolerance = 0.001; 
    double rotationTolerance = 0.1; 
    boolean atTranslation = Math.abs(translationalController.getError()) < translationTolerance;
    boolean atRotation = Math.abs(rotationalController.getError()) < rotationTolerance;
    return atTranslation && atRotation;
}

public boolean isAligned(){
    boolean velocityAligned = true;
    for(SwerveModuleState module : getModuleStates()){
        if(module.speedMetersPerSecond > 0.1){
            velocityAligned = false;
        }
    }
    return velocityAligned && atTargetPose();
}
PathConstraints constraints = new PathConstraints(
        0.25, 1.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

public Command driveToPosePathPl(Pose2d pose){
    return AutoBuilder.pathfindToPose(pose, constraints);
}

public Command alignHubPathpl(){
    return driveToPosePathPl(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0)));
}

// public Rotation2d getTargetRotation(){
//        return new Rotation2d(robotAngle);
// }
}
