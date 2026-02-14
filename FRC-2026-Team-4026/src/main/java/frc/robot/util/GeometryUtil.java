package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeometryUtil { 
    public GeometryUtil (){}
    
    public static double getChassisTranslationSpeeds (ChassisSpeeds chassisSpeeds) {
        double pythagoreumTheorem = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
        return pythagoreumTheorem;
    }
}