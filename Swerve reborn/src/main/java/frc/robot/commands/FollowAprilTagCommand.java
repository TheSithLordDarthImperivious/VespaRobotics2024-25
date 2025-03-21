package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Measure;
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
    private static final double TARGET_DISTANCE_METERS = 0.3; // Stop at 30 cm from the tag
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
        System.out.println("Executing Command...");
        if (visionSubsystem.targetFound()) {
            System.out.println("Target found, trying to see stuff...");

            // Get target transform
            Transform3d targetTransform = visionSubsystem.getTargetTransform();

            // Get rotation3d and quaternion, prepare for angle/yaw
            Quaternion swerveQuaternion = drivetrain.getRotation3d().getQuaternion();

            Quaternion visionQuaternion = targetTransform.getRotation().getQuaternion();

            // Get the measures
            double[] aprilTagDistances = {targetTransform.getMeasureX().in(Meters), targetTransform.getMeasureY().in(Meters), targetTransform.getMeasureZ().in(Meters)};

            double[] swerveQuaternionValues = {swerveQuaternion.getW(), swerveQuaternion.getX(), swerveQuaternion.getY(), swerveQuaternion.getZ()};
            double[] visionQuaternionValues = {visionQuaternion.getW(), visionQuaternion.getX(), visionQuaternion.getY(), visionQuaternion.getZ()};

            // Update quaternion in shuffleboardinterface
            shuffleboardSubsystem.updateQuaternionReadings(swerveQuaternion);

            // Current yaw (calculate using formula)
            double currentYaw = Math.atan2(2 * (swerveQuaternionValues[0] * swerveQuaternionValues[3] + swerveQuaternionValues[0] * swerveQuaternionValues[2]), 1 - 2 * (Math.pow(swerveQuaternionValues[1],2) + Math.pow(swerveQuaternionValues[2],2)));
            // Desired Yaw (calculate using formula)
            double desiredYaw = Math.atan2(2 * (visionQuaternionValues[0] * visionQuaternionValues[3] + visionQuaternionValues[0] * visionQuaternionValues[2]), 1 - 2 * (Math.pow(visionQuaternionValues[1],2) + Math.pow(visionQuaternionValues[2],2)));

            // Compute movement speeds
            double forwardSpeedX = Math.max(0.2, Math.min(1.0, (aprilTagDistances[0] - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
            double forwardSpeedY = Math.max(0.2, Math.min(1.0, (aprilTagDistances[1] - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
            double changeInRotation = desiredYaw - currentYaw; // Scale yaw to rotation (negative to correct direction), and convert to radians

            // Don't forget to normalize to -pi to pi radians
            changeInRotation = ((changeInRotation + Math.PI) % 2*Math.PI) - Math.PI;

            // Convert to radians per second unit
            double rotSpeed = RotationsPerSecond.of(changeInRotation).in(RadiansPerSecond);

            // Timer for one second
            rotationTimer.reset();
            rotationTimer.start();
            
            // Update shuffleboard values
            shuffleboardSubsystem.updateSwerveReadings(0,0,rotSpeed);

            if (rotationTimer.hasElapsed(1)) {
                // Create a movement request
                drivetrain.applyRequest(() -> 
                    new SwerveRequest.FieldCentric()
                        .withVelocityX(forwardSpeedX) // Move forward with forward speed
                        .withVelocityY(forwardSpeedY) // No lateral movement
                        .withRotationalRate(Math.min(rotSpeed,MaxAngularRate)) // Rotate to align with the tag (radians per second)
                );
            } else {
                // Stop robot
                drivetrain.applyRequest(() -> 
                    new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                );
                // Stop timer
                rotationTimer.stop();
            }
            // Update shuffleboard values
            shuffleboardSubsystem.updateAprilTagReadings(visionSubsystem.getTargetID(), 0, desiredYaw, true);
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