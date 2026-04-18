package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

public class Storage {

    public static double currentX = 0;

    public static double currentY = 0;

    public static double currentHeading = 90;

    public static Pose currentPose = new Pose(currentX, currentY, currentHeading);
}