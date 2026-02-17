package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;
@Disabled
public final class TankLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair[] left;
    public PositionVelocityPair[] right;

    public TankLocalizerInputsMessage(List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        this.timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
