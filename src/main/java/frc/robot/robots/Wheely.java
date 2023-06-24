package frc.robot.robots;

import java.util.Map;
import java.util.Optional;

import org.northernforce.gyros.NFRNavX;
import org.northernforce.motors.NFRSparkMax;
import org.northernforce.motors.NFRTalonFX;
import org.northernforce.subsystems.arm.NFRArmMotorExtensionJoint;
import org.northernforce.subsystems.arm.NFRArmMotorExtensionJoint.NFRArmMotorExtensionJointConfiguration;
import org.northernforce.subsystems.arm.NFRSimpleMotorClaw;
import org.northernforce.subsystems.arm.NFRSimpleMotorClaw.NFRSimpleMotorClawConfiguration;
import org.northernforce.subsystems.drive.NFRDrive;
import org.northernforce.subsystems.drive.NFRTankDrive;
import org.northernforce.subsystems.drive.NFRTankDrive.NFRTankDriveConfiguration;
import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.constants.WheelyConstants.*;

public class Wheely implements NFRRobotContainer {
    private final NFRDrive drive;
    private final NFRArmMotorExtensionJoint extensionJoint;
    private final NFRTalonFX extensionMotor; //hack should be getable from joint class in future

    private final NFRSimpleMotorClaw claw;
    public Wheely()
    {
        NFRTankDriveConfiguration driveConfig = new NFRTankDriveConfiguration(
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
        drive = new NFRTankDrive(driveConfig, leftSide, rightSide, navx);

        NFRArmMotorExtensionJointConfiguration extensionConfig =  new NFRArmMotorExtensionJointConfiguration(
            "extension",
            new Transform3d(),
            retractedArmLength,
            extendedArmLength,
            0,
            metersPerRotation,
            false,
            0.05
        );
        TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();
        extensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        extensionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        extensionMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        extensionMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        extensionMotor = new NFRTalonFX(extensionMotorConfig, 11);
        extensionJoint = new NFRArmMotorExtensionJoint(extensionConfig, extensionMotor, Optional.empty(), Optional.empty());
        TalonFXConfiguration clawMotorConfig = new TalonFXConfiguration();
        clawMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        clawMotorConfig.CurrentLimits.SupplyCurrentThreshold = 30;
        clawMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        NFRTalonFX clawMotor = new NFRTalonFX(clawMotorConfig, 13);
        NFRSimpleMotorClawConfiguration clawConfig = new NFRSimpleMotorClawConfiguration("claw")
            .withOpenSpeed(0.6)
            .withCloseSpeed(-0.6)
            .withUseLimitSwitches(true);
        claw = new NFRSimpleMotorClaw(
            clawConfig,
            clawMotor,
            Optional.empty(),
            Optional.of(() -> {
                return clawMotor.getTorqueCurrent().getValue() > 20;
            }),
            Optional.of(() -> {
                return clawMotor.getTorqueCurrent().getValue() > 20;
            }),
            Optional.empty()
        );
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID) {
        if (driverHID instanceof XboxController && manipulatorHID instanceof XboxController)
        {
            XboxController driverController = (XboxController)driverHID;
            XboxController manipulatorController = (XboxController)manipulatorHID;

            drive.setDefaultCommand(drive.getDefaultDriveCommand(
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.07),
                () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.07)
            ));

            // will use when limit switches exist
            // new JoystickButton(manipulatorController, XboxController.Button.kRightBumper.value)
            //     .onTrue(extensionJoint.new ExtendUntilBoolean(forwardIsExtendCoefficient * armSpeed, () -> true)); //TODO make boolean supplier
            // new JoystickButton(manipulatorController, XboxController.Button.kLeftBumper.value)
            //     .onTrue(extensionJoint.new RetractUntilBoolean((-forwardIsExtendCoefficient) * armSpeed, () -> true)); //TODO make boolean supplier
            
            new JoystickButton(manipulatorController, XboxController.Button.kRightBumper.value)
                .onTrue(extensionJoint.getExtendBySpeedCommand(armSpeed));
            new JoystickButton(manipulatorController, XboxController.Button.kLeftBumper.value)
                .onTrue(extensionJoint.getRetractBySpeedCommand(armSpeed));

            new Trigger(() -> manipulatorController.getLeftTriggerAxis() > 0.5)
                .whileTrue(claw.getOpenCommand());
            new Trigger(() -> manipulatorController.getRightTriggerAxis() > 0.5)
                .whileTrue(claw.getCloseCommand());
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
