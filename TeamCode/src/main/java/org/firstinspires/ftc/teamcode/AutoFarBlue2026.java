package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Blue 3 Pickup", group = "Concept")
public class AutoFarBlue2026 extends Autonomous26 {
    @Override
    public boolean setNearOrFar() {
        return false;
    }

    @Override
    public int setPickupTimes() {
        return 3;
    }
}