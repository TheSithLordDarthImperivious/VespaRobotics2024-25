package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardInterface implements Subsystem {

    public ShuffleboardInterface() {
        // No construction required
    }

    public void updateSwerveReadings(double xvol, double yvol, double rot) {
        SmartDashboard.putNumber("X-Velocity", xvol);
        SmartDashboard.putNumber("Y-velocity", yvol);
        SmartDashboard.putNumber("Rotation (radians)", rot);
    }

    public void updateAprilTagReadings(int id, double distance, double yaw, boolean target) {
        SmartDashboard.putNumber("Apriltag ID: ", id);
        SmartDashboard.putNumber("Distance from Apriltag (m)", distance);
        SmartDashboard.putNumber("Apriltag Yaw", yaw);
        SmartDashboard.putBoolean("Target Found", target);
    }
    
    public void addButton(String buttonString, Command buttonCommand){
        SmartDashboard.putData(buttonString, buttonCommand);
    }
}
