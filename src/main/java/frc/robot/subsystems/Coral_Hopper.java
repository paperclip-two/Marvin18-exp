// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.Channel;
import java.security.Timestamp;
import java.time.Instant;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Coral_Hopper extends SubsystemBase {
    private DigitalInput bucketBeamBreak;
    private DigitalInput coralBeamBreak;

    TalonSRX m_coralInOut = new TalonSRX(13);
    TalonSRX m_bucketInOutCoral = new TalonSRX(12);
  /** Creates a new Coral_Hopper. */
  public Coral_Hopper() {
    m_bucketInOutCoral.configFactoryDefault();
    m_coralInOut.configFactoryDefault();
    bucketBeamBreak = new DigitalInput(2);
    coralBeamBreak = new DigitalInput(1);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("CoralBreak",  coralBeamBreak.get());
    //SmartDashboard.putBoolean("", getBucketBreakReading());
  }

  public void setDutyCycle(double dc) {
    m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, dc);
    m_coralInOut.set(TalonSRXControlMode.PercentOutput, dc);
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
            m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, 0);
    });
}

public Command runAgitator(double voltage) {
    return runEnd(() -> {
        if(getCoralBreakReading() == false){
            m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            new WaitCommand(2.0);
            m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        m_bucketInOutCoral.set(TalonSRXControlMode.PercentOutput, 0);
    });
}
}
