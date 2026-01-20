package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Configurable
public class limelight {

    private final Limelight3A limelight;
    private LLResult result;
    private static double timeout=0.2;

    public limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "lL");
    }
    public void init(){
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public int scanAuto() {

        //int returned equals index of green in motif (-1=error.
        // , 0=GPP, 1=PGP, 2=PPG)
        int pattern = -1;
        Timer timeoutTimer = new Timer();
        result = limelight.getLatestResult();
        for (int i = 0; i < 3; i++) {
            limelight.pipelineSwitch(i);
            timeoutTimer.resetTimer();
            while (result.getPipelineIndex() != i && timeoutTimer.getElapsedTimeSeconds()<timeout) {
                if(timeoutTimer.getElapsedTimeSeconds()>0.2){
                    return 0;
                }
                result = limelight.getLatestResult();
            }
            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                pattern = i;
            }
        }

        return pattern;
    }
    public double getDistance(){
        result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            return 67.82807 * Math.pow(result.getTa(), -0.5);
        } else{
            return -1;
        }
    }
    public double getAngle(){
        result = limelight.getLatestResult();
        if(result != null){
            return result.getTxNC();
        } else{
            return -1;
        }
    }

    public Pose relocalize(double heading) {
        result = limelight.getLatestResult();
        limelight.updateRobotOrientation(heading-Math.toRadians(90));
        Pose3D botpose3d = result.getBotpose_MT2();

        double x = botpose3d.getPosition().x;
        double y = botpose3d.getPosition().y;

        Pose2D botpose2d = new Pose2D(DistanceUnit.METER,x , y, AngleUnit.RADIANS, heading);

        return PoseConverter.pose2DToPose(botpose2d, InvertedFTCCoordinates.INSTANCE);
    }
    public boolean isValid(){
        result=limelight.getLatestResult();
        return result != null && result.isValid();
    }

}
