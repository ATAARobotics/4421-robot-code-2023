package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightingSubsystem;


public class LightingCommand extends CommandBase {
    private LightingSubsystem lightingSubsystem;
    private ArmSubsystem armSubsystem;
    private Boolean wantCone;
    public LightingCommand(LightingSubsystem lightingSubsystem, ArmSubsystem armSubsystem, Boolean wantCone) {
        this.lightingSubsystem = lightingSubsystem;
        this.armSubsystem = armSubsystem;
        this.wantCone = wantCone;
    }

    @Override
    public void execute() {
        if (armSubsystem.hasGamePiece()) {
            if (lightingSubsystem.getAliance()) {
                lightingSubsystem.lightRed();
            } else {
                lightingSubsystem.lightBlue();
            }
        } else {
            if (wantCone) {
                lightingSubsystem.lightYellow();
            } else {
                lightingSubsystem.lightPurple();
            }
        }
    }
}