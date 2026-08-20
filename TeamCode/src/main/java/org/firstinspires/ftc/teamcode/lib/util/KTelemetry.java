package org.firstinspires.ftc.teamcode.lib.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KTelemetry {
    Telemetry tele;
    TelemetryManager pTele;
    Telemetry ftcDashboard;
    private String subsystem = null;

    public KTelemetry(Telemetry tele, TelemetryManager pTele, Telemetry ftcDashboard) {
        this.tele = tele;
        this.pTele = pTele;
        this.ftcDashboard = ftcDashboard;
    }

    public KTelemetry(Telemetry tele, TelemetryManager pTele, Telemetry ftcDashboard, String subsys) {
        this.tele = tele;
        this.pTele = pTele;
        this.ftcDashboard = ftcDashboard;

        this.subsystem = subsys;
    }

    public void addData(String name, Object data) {
        if (subsystem == null) {
            tele.addData(name, data);
            pTele.addData(name, data);
            ftcDashboard.addData(name, data);
        } else {
            tele.addData("["+subsystem+"}" + ": " + name, data);
            pTele.addData("["+subsystem+"}" + ": " + name, data);
            ftcDashboard.addData("["+subsystem+"}" + ": " + name, data);
        }
    }

    public void addLine(String line){
        tele.addLine(line);
        pTele.addLine(line);
        ftcDashboard.addLine(line);
    }

    public void update(boolean active) {
        if (active) {
            tele.update();
            pTele.update();
            ftcDashboard.update();
        }
    }

}
