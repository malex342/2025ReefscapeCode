package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private String limelightName;

    public Limelight(String limelightName){
        this.limelightName = limelightName;
    }
}