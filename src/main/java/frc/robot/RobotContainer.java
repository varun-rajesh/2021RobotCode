/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.libs.util.Controller;
import frc.robot.subsystems.ActiveFloor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private Controller driver;
  private Controller manip;
  
  private Drivetrain dt;
  private Intake intake;
  private Pneumatics pneumatics;
  private ActiveFloor floor;
  private Shooter shooter;
  private Climber climber;

  public RobotContainer() {

    driver = new Controller(0, 0.1);
    manip = new Controller(1);
    pneumatics = new Pneumatics();
    climber = new Climber();
    intake = new Intake();

    dt = new Drivetrain();
    dt.setDefaultCommand(new RunCommand(() -> dt.drive(
      driver::getLeftJoystick_X, 
      driver::getLeftJoystick_Y, 
      driver::getRightJoystick_X), dt));
    floor = new ActiveFloor();
    floor.setDefaultCommand(new RunCommand(() -> floor.drive(
      manip::getTriggers, 
      manip::getTriggers), floor));
    shooter = new Shooter();
    shooter.setSetpoint(12000);
    shooter.setDefaultCommand(new RunCommand(shooter::run, shooter));
    climber.setDefaultCommand(new RunCommand(() -> climber.driveClimber(manip::getLeftJoystick_Y), climber));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // driver.getRBButton().whenPressed(() -> MotionOfTheOcean.Recorder.writeOutRecording("/home/lvuser/recording.csv"));
    // driver.getLBButton().whenPressed(() -> MotionOfTheOcean.Executor.loadRecordings("/home/lvuser/recording.csv"));
    driver.getSELECTButton().whenPressed(dt::resetGyro);
    driver.getSELECTButton().whenPressed(dt::reset);
    manip.getLBButton().whenPressed(pneumatics::intakeToggle);
    driver.getSTARTButton().whenPressed(pneumatics::compressorToggle);
    manip.getYButton().whileHeld(intake::runIntake);
    manip.getYButton().whenReleased(intake::stopIntake);
    manip.getAButton().whenPressed(new SequentialCommandGroup(new InstantCommand(pneumatics::hoodExtend), new WaitCommand(0.2), new InstantCommand(pneumatics::hoodLockExtend)));
    manip.getBButton().whenPressed(new SequentialCommandGroup(new InstantCommand(pneumatics::hoodLockRetract), new WaitCommand(0.2), new InstantCommand(pneumatics::hoodRetract)));
    //manip.getAButton().whenPressed(new SequentialCommandGroup(new RunCommand(pneumatics::hoodLockRetract), new WaitCommand(0.2), new RunCommand(pneumatics::hoodExtend)));
    //manip.getBButton().whenPressed(new SequentialCommandGroup(new RunCommand(pneumatics::hoodRetract), new WaitCommand(0.2), new RunCommand(pneumatics::hoodLockExtend)));
    manip.getRBButton().whenPressed(pneumatics::climberLockToggle);
    manip.getXButton().whenPressed(shooter::toggle);
    manip.getRSButton().whileHeld(climber::runSpinner);
    manip.getRSButton().whenReleased(climber::stopSpinner);
  }

}
