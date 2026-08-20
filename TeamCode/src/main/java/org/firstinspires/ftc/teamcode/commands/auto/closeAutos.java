package org.firstinspires.ftc.teamcode.commands.auto;



import com.bylazar.configurables.annotations.Configurable;


import org.firstinspires.ftc.teamcode.commands.TuneablePose;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Configurable
public class closeAutos {
    public static double driveNearRadius = 3;

    public static TuneablePose closeStartPose = new TuneablePose(19.2, 113.3, 180);

    public static Command Close_21_Auto(Drive drive, Intake intake, boolean red) {
        return Commands.sequence(
                DriveCommands.setPose(drive, ()-> closeStartPose.getCorrectPose(red))



        );
    }


}

