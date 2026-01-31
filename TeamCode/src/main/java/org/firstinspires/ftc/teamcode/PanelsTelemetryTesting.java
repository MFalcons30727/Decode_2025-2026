package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Configurable
@TeleOp(name = "PanelsTelemetryTesting", group = "TeleOp")
public class PanelsTelemetryTesting extends OpMode {
    public static double testVarA = 100;
    public static double testVarB = 200;
    @IgnoreConfigurable
    public static double testVarC = 300;
    @IgnoreConfigurable
    TelemetryManager.TelemetryWrapper pTelemetry;

    @Override
    public void init() {
        pTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    }

    @Override
    public void loop() {
        pTelemetry.addData("testVarA", testVarA);
        pTelemetry.addData("testVarB", testVarB);
        pTelemetry.addData("testVarC", testVarC);
    }
}
