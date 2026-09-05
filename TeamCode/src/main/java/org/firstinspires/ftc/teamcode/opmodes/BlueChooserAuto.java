package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name = "Blue_Chooser_Auto", group = "Auto")
public class BlueChooserAuto extends SelectableAutoOpMode {

    public BlueChooserAuto() {
        super("Select a Blue Auto", s -> {
            s.add("Blue - Default", rc -> rc.getAutoCommand(OpModeConstants.BLUE_AUTO));
            s.add("Blue - Alternative", rc -> rc.getAutoCommand(OpModeConstants.BLUE_AUTO));
        });
    }

    @Override
    protected RobotContainer createRobotContainer() {
        return new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2,
                OpModeConstants.BLUE_AUTO, false);
    }
}