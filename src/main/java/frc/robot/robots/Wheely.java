package frc.robot.robots;

import java.util.Map;

import org.northernforce.gyros.NFRNavX;
import org.northernforce.motors.NFRSparkMax;
import org.northernforce.subsystems.drive.NFRDrive;
import org.northernforce.subsystems.drive.NFRTankDrive;
import org.northernforce.subsystems.drive.NFRTankDrive.NFRTankDriveConfiguration;
import org.northernforce.util.NFRRobotContainer;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Wheely implements NFRRobotContainer {
    private final NFRDrive drive;
    public Wheely()
    {
        NFRTankDriveConfiguration config = new NFRTankDriveConfiguration(
            "drive",
            22.3125,
            11.39,
            Math.PI * 6,
            0.6,
            Units.lbsToKilograms(40),
            14,
            Math.toRadians(270),
            0,
            1,
            false,
            false,
            false,
            DCMotor.getNEO(2)
        );
        NFRSparkMax leftSide = new NFRSparkMax(MotorType.kBrushless, 1, 3);
        NFRSparkMax rightSide = new NFRSparkMax(MotorType.kBrushless, 2, 4);
        NFRNavX navx = new NFRNavX();
        leftSide.setInverted(false);
        rightSide.setInverted(true);
        drive = new NFRTankDrive(config, leftSide, rightSide, navx);
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID) {
        if (driverHID instanceof XboxController && manipulatorHID instanceof XboxController)
        {
            XboxController driverController = (XboxController)driverHID;
            drive.setDefaultCommand(drive.getDefaultDriveCommand(
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.07),
                () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.07)
            ));
        }
    }
    @Override
    public Map<String, Command> getAutonomousOptions() {
        return Map.of("Haha! You got no autonomous!", new InstantCommand());
    }
    @Override
    public Map<String, Pose2d> getStartingLocations() {
        return Map.of("Haha! I don't care where you start", new Pose2d());
    }
}
