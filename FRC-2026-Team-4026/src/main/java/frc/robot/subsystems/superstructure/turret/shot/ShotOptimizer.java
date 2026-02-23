package frc.robot.subsystems.superstructure.turret.shot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;

import org.littletonrobotics.junction.Logger;

public class ShotOptimizer {
    public record OptimalShot(Rotation2d turretYaw, Rotation2d turretPitch, double turretVel) {}

    private static double G = 9.80665;
    private Drive drive;
    private Pose2d robot;
    private Translation3d target;
    
    public ShotOptimizer(){} 

    double robotVelocity = RobotState.getInstance().getChassisVelocity();
    ChassisSpeeds robotFieldVelocity = RobotState.getInstance().getFieldVelocity();

    // 1. Project robot into future (latency comp)
    Pose2d future =
        robot.exp(
            new Twist2d(
                drive.getChassisSpeeds().vxMetersPerSecond * ShooterConstants.LOOK_AHEAD_SECONDS,
                drive.getChassisSpeeds().vyMetersPerSecond * ShooterConstants.LOOK_AHEAD_SECONDS,
                drive.getChassisSpeeds().omegaRadiansPerSecond * ShooterConstants.LOOK_AHEAD_SECONDS));
    Pose3d turretPos = new Pose3d(future).transformBy(ShooterConstants.ROBOT_TO_TURRET);

    // 2. Calculate field relative direction vector 
    Translation3d trajectoryVector = target.minus(turretPos.getTranslation());

    // 3. Calculate field relative turret velocity
    // 	v_t = v_r(field) + w_r(field) x r_t(field)
    Translation2d turret_radius_perp =
        new Translation2d(
                -ShooterConstants.ROBOT_TO_TURRET.getY(), ShooterConstants.ROBOT_TO_TURRET.getX())
            .rotateBy(future.getRotation());
    Translation2d turretFieldVelocity =
        new Translation2d(robotFieldVelocity.vxMetersPerSecond, robotFieldVelocity.vyMetersPerSecond)
            .plus(turret_radius_perp.times(robotFieldVelocity.omegaRadiansPerSecond));

    double funnelHorizontalDistance =
        AllianceFlipUtil.apply(FieldConstants.Hub.nearFace)
            .getTranslation()
            .getDistance(future.getTranslation());

    // NOTE: From here on object creation is minimized for sampling performance
    double targetX = trajectoryVector.getX();
    double targetY = trajectoryVector.getY();
    double targetZ = trajectoryVector.getZ();

    double turretVelocityX = turretFieldVelocity.getX();
    double turretVelocityY = turretFieldVelocity.getY();
    double turretVelocityZ = 0.0;

    double minTime = 0.0;
    double maxTime = 6.0;
    int samples = 100;

    double optimalPitch = ShooterConstants.OPTIMAL_PITCH.in(Radians);
    double bestCost = Double.MAX_VALUE;
    OptimalShot shot = new OptimalShot(Rotation2d.kZero, Rotation2d.kZero, 0.0);
    double clearance = ShooterConstants.hubFunnelClearance.in(Meters);

    // 4. Samples possible time of flight values, minimizes cost function:
    //
    // C(t) = w1 * v(t)^2 + w2 * (pitch(t) - optimal_pitch)^2 + w3 * t
    //
    // Prioritizes:
    // 	- low speed
    // 	- close to mid angle
    // 	- quick tof
    //
    // * Also obeys for funnel constraint by computing shot height at hub front-face
    //  horizontal displacement with chosen parameters and comparing to tunable clearance value
    //

    public OptimalShot apply() {
        //Translation2d localEstimate = RobotState.getInstance().getHubLocalizedRobotPose();
        var robot = RobotState.getInstance().getEstimatedRobotPose();
        var target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
        var initialDistance = robot.getTranslation().getDistance(target.toTranslation2d());
        //localestimate != null. Should this be in params?
        //if (initialDistance > 0.4 && initialDistance < 3.0) {
          //  robot = localEstimate.toPose2d();
        //}
        return shot; 
       }  
    
    public void sampleTrajectory() {
    for (int i = 0; i < samples; i++) {
      double time = minTime + i * (maxTime - minTime) / (samples - 1);

      double xFieldVelocity = targetX / time;
      double yFieldVelocity = targetY / time;
      double zFieldVelocity = (targetZ + 0.5 * G * time * time) / time;

      double relativeVelocityX = xFieldVelocity - turretVelocityX;
      double relativeVelocityY = yFieldVelocity - turretVelocityY;
      double relativeVelocityZ = zFieldVelocity - turretVelocityZ;

      double hypVelocityXY = Math.hypot(relativeVelocityX, relativeVelocityY);
      double pitchRadians = Math.atan2(relativeVelocityZ, hypVelocityXY);
      double velocity = Math.hypot(relativeVelocityZ, hypVelocityXY);

      double velocityCost = velocity * velocity;
      double pitchCost = Math.pow(pitchRadians - optimalPitch, 2);
      double timeCost = time;

      double cost =
          (velocityCost * ShooterConstants.TRAJECTORY_WEIGHTS.get(0, 0))
              + (pitchCost * ShooterConstants.TRAJECTORY_WEIGHTS.get(1, 0))
              + (timeCost * ShooterConstants.TRAJECTORY_WEIGHTS.get(2, 0));

      double timeFunnel = funnelHorizontalDistance / Math.hypot(xFieldVelocity, yFieldVelocity);
      double funnelY = zFieldVelocity * timeFunnel - 0.5 * G * timeFunnel * timeFunnel;

      if (funnelY > clearance && cost < bestCost) {
        bestCost = cost;
        shot = new OptimalShot(new Rotation2d(relativeVelocityX, relativeVelocityY), new Rotation2d(pitchRadians), velocity);
      }
    }
  }
    

    // Logger.recordOutput("OptimalShot/horizontalDistance", funnelHorizontalDistance);
    // Logger.recordOutput("OptimalShot/velocity", shot.turretVel());
    // Logger.recordOutput("OptimalShot/pitch", shot.turretPitch());
    // Logger.recordOutput("OptimalShot/yaw", shot.turretYaw());
    // Logger.recordOutput("OptimalShot/clearanceInches", clearance);

  }


