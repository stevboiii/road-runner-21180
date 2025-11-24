package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Near Red 1 Gate 3 Rows", group = "Concept")
public class AutoNearRed2026 extends AutoNearBlue2026 {
    @Override
    // the only difference against AutoNearBlue2026 is the side flag.
    public int setSide() {
        return -1;
    }
}