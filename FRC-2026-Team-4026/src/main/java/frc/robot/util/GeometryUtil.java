package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil { 
    public GeometryUtil (){}
    
    public static double getChassisTranslationSpeeds (ChassisSpeeds chassisSpeeds) {
        double pythagoreumTheorem = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
        return pythagoreumTheorem;
    }
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, Rotation2d.kZero);
    }





  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }
}