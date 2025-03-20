package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShuffleboardInterface;
import frc.robot.subsystems.Vision;

public class FollowAprilTagCommand extends Command {
    private final Vision visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final ShuffleboardInterface shuffleboardSubsystem = new ShuffleboardInterface();
    private static final double TARGET_DISTANCE_METERS = 1.0; // Stop at 1 meter from the tag
    private Timer rotationTimer = new Timer();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public FollowAprilTagCommand(Vision visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        System.out.println("Activated.");
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        if (visionSubsystem.targetFound()) {
            System.out.println("Target found, trying to see stuff...");
            // Get rotation3d and quaternion, prepare for angle/yaw
            Quaternion swerveQuaternion = drivetrain.getRotation3d().getQuaternion();

            // Get quaternion components
            double w = swerveQuaternion.getW();
            double x = swerveQuaternion.getX();
            double y = swerveQuaternion.getY();
            double z = swerveQuaternion.getZ();

            // Update quaternion in shuffleboardinterface
            shuffleboardSubsystem.updateQuaternionReadings(swerveQuaternion);

            // Current yaw (calculate using formula)
            double currentYaw = Math.atan2(2 * (w * z + w * y), 1 - 2 * (Math.pow(x,2) + Math.pow(y,2)));

            // Get distance and angle to the AprilTag
            double targetYaw = visionSubsystem.getTargetYaw(); // Degrees
            double targetDistance = visionSubsystem.getTargetDistance(); // Meters

            // Compute movement speeds
            double forwardSpeed = Math.max(0.2, Math.min(1.0, (targetDistance - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
            double changeInRotation = -targetYaw * (Math.PI / 180) - currentYaw; // Scale yaw to rotation (negative to correct direction), and convert to radians

            // Don't forget to normalize to -pi to pi radians
            changeInRotation = ((changeInRotation + Math.PI) % 2*Math.PI) - Math.PI;

            // Convert to radians per second unit
            double rotSpeed = RotationsPerSecond.of(changeInRotation).in(RadiansPerSecond);

            // Timer for one second
            rotationTimer.reset();
            rotationTimer.start();

            if (rotationTimer.get() < 1.0) {
                // Create a movement request
                drivetrain.applyRequest(() -> 
                    new SwerveRequest.FieldCentric()
                        .withVelocityX(forwardSpeed) // Move forward with forward speed
                        .withVelocityY(0) // No lateral movement
                        .withRotationalRate(Math.min(rotSpeed,MaxAngularRate)) // Rotate to align with the tag (radians per second)
                );
            } else {
                drivetrain.applyRequest(() -> 
                    new SwerveRequest.FieldCentric()
                        .withVelocityX(0) // Move forward with forward speed
                        .withVelocityY(0) // No lateral movement
                        .withRotationalRate(0) // Rotate to align with the tag (radians per second)
                );
            }
            // Update shuffleboard values
            shuffleboardSubsystem.updateSwerveReadings(forwardSpeed,0,rotSpeed);
            shuffleboardSubsystem.updateAprilTagReadings(visionSubsystem.getTargetID(), targetDistance, targetYaw, true);
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("Finished...");
        double targetDistance = visionSubsystem.getTargetDistance();
        return targetDistance > 0 && targetDistance <= TARGET_DISTANCE_METERS;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Somehow, it ended");
        drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()); // Stop the robot
    }
}