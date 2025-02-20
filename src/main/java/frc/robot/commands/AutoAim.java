package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants.*;

public class AutoAim extends Command{
  private SwerveDrive swerve;
  private PIDController rotateController;

  private double start;
  private double end;

  private double tx;
  private double current;


  public AutoAim(double tx, SwerveDrive swerve){
    this.tx = tx;
    this.swerve = swerve;
    addRequirements(swerve);

    rotateController = new PIDController(
      .01, 
      .01, 
      .01
    );

    rotateController.reset();
    rotateController.setTolerance(2);
  }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
    rotateController.enableContinuousInput(0, 360);
    start = 0;
    end = start + tx;
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    rotateController.setSetpoint(end);
    current = swerve.getGyro().getYaw();
    double rotationSpeed = rotateController.calculate(current, end);
    ChassisSpeeds radial = new ChassisSpeeds(0, 0, rotationSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve.getModuleStates(), MAX_ROTATE_SPEED * 0.3);
    swerve.drive(radial, MAX_DRIVE_SPEED); 
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    swerve.stopModules();
   }

// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotateController.atSetpoint();
  }
}
