package frc.robot.subsystems;

import java.util.Collection;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MusicSubsystem extends SubsystemBase {
  TalonFX driveMotor;
  TalonFX driveMotor2;
  TalonFX driveMotor3;
  TalonFX driveMotor4;
  TalonFX rotationMotor;
  TalonFX rotationMotor2;
  TalonFX rotationMotor3;
  TalonFX rotationMotor4;
  Collection<TalonFX> wheels;
  Orchestra music;
  public MusicSubsystem() {
    driveMotor = new TalonFX(Constants.DRIVE_MOTORS_ID[0]);
    driveMotor2 = new TalonFX(Constants.DRIVE_MOTORS_ID[1]);
    driveMotor3 = new TalonFX(Constants.DRIVE_MOTORS_ID[2]);
    driveMotor4 = new TalonFX(Constants.DRIVE_MOTORS_ID[3]);
    rotationMotor = new TalonFX(Constants.ROTATION_MOTORS_ID[0]);
    rotationMotor2 = new TalonFX(Constants.ROTATION_MOTORS_ID[1]);
    rotationMotor3 = new TalonFX(Constants.ROTATION_MOTORS_ID[2]);
    rotationMotor4 = new TalonFX(Constants.ROTATION_MOTORS_ID[3]);
    wheels.add(driveMotor);
    wheels.add(driveMotor2);
    wheels.add(driveMotor3);
    wheels.add(driveMotor4);
    wheels.add(rotationMotor);
    wheels.add(rotationMotor2);
    wheels.add(rotationMotor3);
    wheels.add(rotationMotor4);
    music = new Orchestra(wheels, "./music.chrp");
  }

  public void musicPlay() {
    music.play();
  }
  public void musicPause() {
    music.pause();
  }
  public void musicStop() {
    music.stop();
  }
  public BooleanSupplier isPlaying() {
    return () -> music.isPlaying();
  }
}
