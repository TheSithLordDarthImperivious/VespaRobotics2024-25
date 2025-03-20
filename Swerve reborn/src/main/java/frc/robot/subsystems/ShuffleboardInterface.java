package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardInterface implements Subsystem {

    public ShuffleboardInterface() {
        SmartDashboard.putString("Initalized", "Initalized Shuffleboard Interface");
        System.out.println("Initalized Shuffleboard Interface");
        // No need to construct anything
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

    public void updateQuaternionReadings(Quaternion quaternion) {
        SmartDashboard.putNumber("Quaternion W (scalar real number)", quaternion.getW());
        SmartDashboard.putNumber("Quaternion X (directional imaginary number)", quaternion.getX());
        SmartDashboard.putNumber("Quaternion Y (directional imaginary number)", quaternion.getY());
        SmartDashboard.putNumber("Quaternion Z (directional imaginary number)", quaternion.getZ());
    }
    
    public void addButton(String buttonString, Command buttonCommand) {
        SmartDashboard.putData(buttonString, buttonCommand);
    }
}
