package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@TeleOp
public class FlywheelTuningWithPanels extends OpMode {
    @IgnoreConfigurable
    private DcMotor indexer;
    @IgnoreConfigurable
    private DcMotorEx shooter;
    private double highVelocity = 1250; // Set this to the target velocity
    private double lowVelocity = 1000; // Set this to the drop normally seen immediately after shooting
    @IgnoreConfigurable
    private double curTargetVelocity = highVelocity;
    private double F = 12;
    private double P = 190;
    @IgnoreConfigurable
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    @IgnoreConfigurable
    private int stepIndex = 1;
    @IgnoreConfigurable
    TelemetryManager.TelemetryWrapper pTelemetry; // this is a wrapper so telemetry shows on driver hub and in panels for graphing

    @Override
    public void init() {

        indexer = hardwareMap.get(DcMotor.class, "indexer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        pTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        pTelemetry.addLine("Init complete");
        pTelemetry.update();
    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad2.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // cycle through the different step sizes in a looping fashion
        }

        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if(gamepad2.left_bumper) {
            indexer.setPower(1);
        }
        else {
            indexer.setPower(0);
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        shooter.setVelocity(curTargetVelocity);

        double curVelocity = shooter.getVelocity();
        double error = curTargetVelocity - curVelocity;

        pTelemetry.addData("Target Velocity", curTargetVelocity);
        pTelemetry.addData("Current Velocity", "%.2f", curVelocity);
        pTelemetry.addData("Error", "%.2f", error);
        pTelemetry.addLine("-------------------------------");
        pTelemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        pTelemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        pTelemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        pTelemetry.update();
    }
}