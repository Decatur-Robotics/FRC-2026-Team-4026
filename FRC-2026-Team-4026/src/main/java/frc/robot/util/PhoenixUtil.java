// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  /** Refresh all registered signals. */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      BaseStatusSignal.refreshAll(canivoreSignals);
    }
    if (rioSignals.length > 0) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
  }


public static SwerveModuleConstants regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return moduleConstants;

        // Apply simulation-specific adjustments to module constants
        return moduleConstants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(new Slot0Configs()
                        .withKP(70)
                        .withKI(0)
                        .withKD(4.5)
                        .withKS(0)
                        .withKV(1.91)
                        .withKA(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(16.0)
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}