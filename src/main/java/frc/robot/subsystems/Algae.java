// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Algae extends SubsystemBase {
    private DigitalInput bucketBeamBreak;
    private DigitalInput coralBeamBreak;

    TalonSRX algae = new TalonSRX(Constants.CAN_IDS.ALGAE_MECHANISM.ALGAE_MECH_MC);
  /** Creates a new Coral_Hopper. */
  public Algae() {
    algae.configFactoryDefault();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("", getBucketBreakReading());
  }

  public void setDutyCycle(double dc) {
    algae.set(TalonSRXControlMode.PercentOutput, dc);
}

public Command runAlgaeWheels(double percent) {
    return runEnd(() -> {
        algae.set(TalonSRXControlMode.PercentOutput, percent);

    },
    () -> {
        algae.set(TalonSRXControlMode.PercentOutput, 0);
    });
}
}
