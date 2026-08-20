package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends CommandOpMode {

    private RobotContainer robotContainer;
    private KTelemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    List<LynxModule> allHubs;

    @Override
    public void robotInit() {
        robotTelemetry = new KTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry(), FtcDashboard.getInstance().getTelemetry());
        robotContainer = new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2, OpModeConstants.TELEOP, false); //Uses heavily modified untested hardware
        timer.start();
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotContainer.periodic();

        robotTelemetry.addData("Loop Time", 1.0 / (timer.get() - previousTime));
        robotTelemetry.update(robotContainer.getTelemetryActive());
        previousTime = timer.get();

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}