// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Hopper extends SubsystemBase {
    private AnalogInput bucketIR = new AnalogInput(1);
    private AnalogInput coralIR = new AnalogInput(0);

    TalonSRX agitator = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.AGITATOR);
    TalonSRX coralIntake = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_INTAKE);
  /** Creates a new Coral_Hopper. */
  public Hopper() {
    coralIntake.configFactoryDefault();
    agitator.configFactoryDefault();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral/CoralIR", coralIR.getValue());
    SmartDashboard.putNumber("Bucket/BucketIR", bucketIR.getValue());

    //SmartDashboard.putBoolean("", getBucketBreakReading());
  }

  public void setDutyCycle(double dc) {
    coralIntake.set(TalonSRXControlMode.PercentOutput, dc);
    agitator.set(TalonSRXControlMode.PercentOutput, dc);
}

public Command runCoralAgitator(double percentOut) {
    return runEnd(() -> {
        agitator.set(TalonSRXControlMode.PercentOutput, percentOut);
    },
    () -> {
        agitator.set(TalonSRXControlMode.PercentOutput, 0);
    });
}

public Command runIntake(double percentOut) {
    return runEnd(() -> {
     coralIntake.set(TalonSRXControlMode.PercentOutput, percentOut);
    },
    () -> {
        coralIntake.set(TalonSRXControlMode.PercentOutput, 0);
    });
}
}
