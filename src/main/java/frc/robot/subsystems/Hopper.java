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

public class Hopper extends SubsystemBase {
    private DigitalInput bucketBeamBreak;
    private DigitalInput coralBeamBreak;

    TalonSRX coralIntakeMotor = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_INTAKE_MC);
    TalonSRX bucketMotor = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_HOLDER_MC);
  /** Creates a new Coral_Hopper. */
  public Hopper() {
    bucketMotor.configFactoryDefault();
    coralIntakeMotor.configFactoryDefault();
    bucketBeamBreak = new DigitalInput(Constants.DIO_IDS.BUCKET_BEAMBREAK);
    coralBeamBreak = new DigitalInput(Constants.DIO_IDS.INTAKE_BEAMBREAK);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CoralBreak", coralBeamBreak.get());
    //SmartDashboard.putBoolean("", getBucketBreakReading());
  }

  public void setDutyCycle(double dc) {
    bucketMotor.set(TalonSRXControlMode.PercentOutput, dc);
    coralIntakeMotor.set(TalonSRXControlMode.PercentOutput, dc);
}

public boolean getCoralBreakReading(){
    return coralBeamBreak.get(); 
 }

 public boolean getBucketBreakReading(){
    return bucketBeamBreak.get();
 }
  
  public Command runVoltageUntilIRReading(double voltage) {
    return runEnd(() -> {
        if(getBucketBreakReading() == true){
            bucketMotor.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            bucketMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        bucketMotor.set(TalonSRXControlMode.PercentOutput, 0);
    });
}

public Command runAgitator(double voltage) {
    return runEnd(() -> {
        if(getCoralBreakReading() == false){
            bucketMotor.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            new WaitCommand(2.0);
            bucketMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        bucketMotor.set(TalonSRXControlMode.PercentOutput, 0);
    });
}
}
