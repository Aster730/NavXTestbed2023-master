/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import Xbox Controller and related buttons and axes
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

// import limelight
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.AimTarget;


// import commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import RevDrivetrain subsystem
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.NavX.AutoBalance;

// import constants
import static frc.robot.Constants.*;

//import NavX 
import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // Drive Controller
  private XboxController xbox = new XboxController(kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // limelight subsystem
  private final Limelight limelight = new Limelight();

  //NavX setup
  private final AHRS ahrs = new AHRS();

  /* --- Default Commands --- */

  // drive with controller 
  private Command manualDrive = new RunCommand(
    
  // drive motors run in tank drive based on joystick inputs
    () -> rDrive.getDifferentialDrive().tankDrive (
      rDrive.deadband(-xbox.getRawAxis(kLeftY.value), kPercentDeadband), 
      rDrive.deadband(xbox.getRawAxis(kRightY.value), kPercentDeadband),
      true
      ),
    rDrive
  );

  /* --- Container for the robot --- contains subsystems, OI devices, and commands */
  public RobotContainer() {

    // configure the button bindings
    configureButtonBindings();

    // run manualDrive and moveArm as the default commands
    rDrive.setDefaultCommand(manualDrive);
  }

  private void configureButtonBindings() {

    // vision correction
    new JoystickButton(xbox, kX.value)
    .whileTrue(new AimTarget(limelight, rDrive));

    //auto balance, only used when on unbalanced charge station
    new JoystickButton(xbox, kY.value)
    .whileTrue(new AutoBalance());

  }

  public void init() {
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();
  }

  public void periodic() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
