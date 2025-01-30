package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants.FConstants;
import org.firstinspires.ftc.teamcode.Constants.LConstants;
@Disabled
@Autonomous
public class Spec_Net_Hang_Red extends OpMode{
    private Follower follower = null;

    private PathChain A = null;
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        generatePath();
    }

    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            follower.followPath(A);
        }
    }

    public void generatePath(){
        A = follower.pathBuilder()

                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(136.171, 67.137, Point.CARTESIAN),
                                new Point(103.433, 67.137, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(103.433, 67.137, Point.CARTESIAN),
                                new Point(128.580, 45.786, Point.CARTESIAN),
                                new Point(85.166, 29.180, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(85.166, 29.180, Point.CARTESIAN),
                                new Point(85.404, 22.537, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(85.404, 22.537, Point.CARTESIAN),
                                new Point(97.740, 29.417, Point.CARTESIAN),
                                new Point(134.985, 10.201, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(134.985, 10.201, Point.CARTESIAN),
                                new Point(61.443, 47.921, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90)).build();
    }
}
