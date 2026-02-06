package frc.robot.subsystems.superstructure.hopper;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Ports;

public class Hopper {

    private TalonFX motorRight, motorLeft;

public void setVoltage (double voltage) {
        motorLeft.setVoltage(voltage);
        motorRight.setVoltage(voltage);
        motorRight.setControl(new Follower(Ports.HOPPER_MOTOR_LEFT, MotorAlignmentValue.Opposed));
}    

}
