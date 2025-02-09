// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Hopper extends SubsystemBase {
    private AnalogInput bucketIR;
    private AnalogInput coralIR;

    TalonSRX agitator;
    TalonSRX coralIntake;
  /** Creates a new Coral_Hopper. */
  public Hopper() {
    bucketIR = new AnalogInput(1);
    coralIR = new AnalogInput(2);
    agitator = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.AGITATOR);
    coralIntake = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_INTAKE);

    coralIntake.configFactoryDefault();
    agitator.configFactoryDefault();
    coralIR.setAverageBits(4);
    coralIR.setOversampleBits(4);
    bucketIR.setOversampleBits(4);
    bucketIR.setAverageBits(4);


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CoralIR", getCoralIRReading());
    SmartDashboard.putNumber("BucketIR", getBucketIRReading());
    runAgitatorWhenReading(1);
    //SmartDashboard.putBoolean("", getBucketIRReading());
  }

  public void setDutyCycle(double dc) {
    coralIntake.set(TalonSRXControlMode.PercentOutput, dc);
    agitator.set(TalonSRXControlMode.PercentOutput, dc);
}

public double getCoralIRReading(){
    return coralIR.getAverageVoltage();
 }

 public double getBucketIRReading(){
    return bucketIR.getAverageVoltage();
 }
 
  public Command runIntakeUntilIRReading(double voltage) {
    return runEnd(() -> {
        if(getBucketIRReading() > DynamicConstants.IRThresholds.bucketIRthreshold){
            coralIntake.set(TalonSRXControlMode.PercentOutput, voltage);
        }
    },
    () -> {
        coralIntake.set(TalonSRXControlMode.PercentOutput, 0);
    });
} 

public Command runAgitatorWhenReading(double voltage) {
    return runEnd(() -> {
        if(getCoralIRReading() < DynamicConstants.IRThresholds.bucketIRthreshold){
            agitator.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            new WaitCommand(2.0);
            agitator.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        agitator.set(TalonSRXControlMode.PercentOutput, 0);
    });
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
