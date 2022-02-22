// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private WPI_TalonFX leftArm;
  private WPI_TalonFX rightArm;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
    leftArm = new WPI_TalonFX(13);
    rightArm = new WPI_TalonFX(12);

    leftArm.follow(rightArm, FollowerType.PercentOutput);
    leftArm.setNeutralMode(NeutralMode.Brake);
    rightArm.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Suggested kF", this::getSuggestedKF, null);
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Target", null, this::setTarget);
    builder.addDoubleProperty("kF", null, this::setKf);
    builder.addDoubleProperty("kP", null, this::setKp);
    builder.addDoubleProperty("kI", null, this::setKi);
    builder.addDoubleProperty("iZone", null, this::setiZone);
  }

  public void extendArm() {
    rightArm.set(ControlMode.PercentOutput, 0.5);
  }

  public void retractArm() {
    rightArm.set(ControlMode.PercentOutput, -0.5);
  }

  public void stopArm() {
    rightArm.stopMotor();
  }

  public double getSuggestedKF() {
    return (rightArm.get() * 1023) / getVelocity();
  }

  public double getVelocity() {
    return rightArm.getSelectedSensorVelocity();
  }

  public void setKf(double newK) {
    rightArm.config_kF(0, newK);
  }

  public void setKp(double newP) {
    rightArm.config_kP(0, newP);
  }

  public void setKi(double newI) {
    rightArm.config_kI(0, newI);
  }

  public void setiZone(double zone) {
    rightArm.config_IntegralZone(0, zone);
  }

  public void setTarget(double pos) {
    rightArm.set(ControlMode.Position, pos);
  }

  public double getPosition() {
    return rightArm.getSelectedSensorPosition();
  }
}
