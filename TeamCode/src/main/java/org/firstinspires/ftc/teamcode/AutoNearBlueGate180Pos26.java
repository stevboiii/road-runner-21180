package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Near Blue Gate Pos180", group = "Concept")
public class AutoNearBlueGate180Pos26 extends AutoNearBlueGate26 {

    @Override
    public void setStartPosition() {
        startPose = new Pose2d(
                (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - 1.5), // add additional 0.5 inch according to testing
                (leftOrRight * (4 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH -1.0)), Math.toRadians(180.0)
        );
    }
}