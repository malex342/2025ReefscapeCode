// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax rotateMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder rotateEncoder;

  private PIDController rotateController;

  private AnalogInput absoluteEncoder;
  private boolean absEncoderReverse;

  private double encoderOffset;

  private SparkMaxConfig driveConfig;
  private SparkMaxConfig rotateConfig;
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int rotateID, int magEncoderPort, boolean invertRotate, boolean invertDrive, double encoderOffset, double[] PID_values) {
    
    // System.out.println("Drive: " + driveID + " rotate: " + rotateID);
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new SparkMax(rotateID, MotorType.kBrushless);
    driveConfig = new SparkMaxConfig();
    rotateConfig = new SparkMaxConfig();
    driveConfig.smartCurrentLimit(60);
    rotateConfig.smartCurrentLimit(60);
    

    this.encoderOffset = encoderOffset;

    driveConfig.idleMode(IdleMode.kBrake);
    driveMotor.setInverted(invertDrive);

    rotateConfig.idleMode(IdleMode.kBrake);
    rotateMotor.setInverted(invertRotate);

    driveEncoder = driveMotor.getEncoder();
    driveConfig.encoder.positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION);
    driveConfig.encoder.velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION);

    rotateEncoder = rotateMotor.getEncoder();
    rotateConfig.encoder.positionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION);
    rotateConfig.encoder.velocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

    absoluteEncoder = new AnalogInput(magEncoderPort);

    rotateController = new PIDController(PID_values[0], PID_values[1], PID_values[2]);
    rotateController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoder();
  }

  /**
   * @return Drive encoder position in meters.
   */
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  /**
   * @return Rotate encoder position in meters.
   */
  public double getRotatePosition() {
    return rotateEncoder.getPosition();
  }

  /**
   * @return Drive encoder velocity in meters/second
   */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }
  /**
   * @return Rotate encoder velocity in radians/second.
   */
  public double getRotateVelocity() {
    return rotateEncoder.getVelocity();
  }

  /**
   * @return Absolute encoder angle in radians with offset removed.
   */
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;
    angle -= encoderOffset;
    angle %= 2 * Math.PI;

    return angle * (absEncoderReverse ? -1.0 : 1.0);
  }

  /** 
   * @return Encoder angles with offsets.
  */
  public double getOffsets() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;

    return angle * (absEncoderReverse ? -1.0 : 1.0); 
  }

  public void resetEncoder() {
    driveEncoder.setPosition(0);
    rotateEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public void stop() {
    driveMotor.set(0);
    rotateMotor.set(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition()));
  }

  /**
   * Sets the calculated speed of the drive and rotate motors.
   * @param state is optimized for minimizing movement 
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_DRIVE_SPEED);
    rotateMotor.set(rotateController.calculate(getRotatePosition(), state.angle.getRadians()));
  }

  public void setCoastMode() {
    driveConfig.idleMode(IdleMode.kCoast);
    rotateConfig.idleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    driveConfig.idleMode(IdleMode.kBrake);
    rotateConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotate encoder" + rotateEncoder, rotateEncoder.getPosition());
  }
}
