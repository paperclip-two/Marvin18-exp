// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX motor;
  private double positionCoefficient = 1.0/116.666666667;
  private ProfiledPIDController profiledPIDController;
  private double gearRatio = 1.0/100.0 * 180/40;
    private double nativePositionToDegrees = 360.0 / 2048 * gearRatio;

  public Arm() {
    motor = new TalonFX(1);
    motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(null)
    .withReverseSoftLimitEnable(true)
    .withReverseSoftLimitThreshold(null));
    motor.getConfigurator().apply(new VoltageConfigs()
    .withPeakForwardVoltage(2)
    .withPeakReverseVoltage(-2));
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setPosition(0);

    profiledPIDController = new ProfiledPIDController(0, 0, 0, null);
    profiledPIDController.setTolerance(0.005);

    SmartDashboard.putData(profiledPIDController);
  }

  public void setDutyCycle(double d){
    motor.set(d);
  }

  public Command runVoltage(double voltage) {
    return runEnd(() -> {
      motor.setVoltage(voltage);
    }, () -> {
      motor.set(0);
    });
  }

  public Command setTurretPosition(double position) {
    return runEnd(() -> {
        motor.set(profiledPIDController.calculate(motor.getPosition().getValueAsDouble(), position / positionCoefficient));
    },
    () -> {
        motor.set(0);
    });
}

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
