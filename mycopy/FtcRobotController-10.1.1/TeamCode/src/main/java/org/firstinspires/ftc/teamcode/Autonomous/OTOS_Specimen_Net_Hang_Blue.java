package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OTOS_Specimen_Net_Hang_Blue extends OpMode {
        public OTOS_Specimen_Net_Hang_Blue() {
            PathBuilder builder = new PathBuilder();

            builder
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(8.303, 84.929, Point.CARTESIAN),
                                    new Point(23.486, 85.166, Point.CARTESIAN)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            // Line 2
                            new BezierLine(
                                    new Point(23.486, 85.166, Point.CARTESIAN),
                                    new Point(39.381, 75.677, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierLine(
                                    new Point(39.381, 75.677, Point.CARTESIAN),
                                    new Point(31.789, 98.214, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierLine(
                                    new Point(31.789, 98.214, Point.CARTESIAN),
                                    new Point(63.341, 119.328, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .addPath(
                            // Line 5
                            new BezierLine(
                                    new Point(63.341, 119.328, Point.CARTESIAN),
                                    new Point(11.862, 127.394, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .addPath(
                            // Line 6
                            new BezierLine(
                                    new Point(11.862, 127.394, Point.CARTESIAN),
                                    new Point(68.086, 97.265, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .addPath(
                            // Line 7
                            new BezierLine(
                                    new Point(68.086, 97.265, Point.CARTESIAN),
                                    new Point(68.086, 92.758, Point.CARTESIAN)
                            )
                    )
                    .setTangentHeadingInterpolation();
        }

        @Override
        public void init() {

        }

        @Override
        public void loop() {

        }
}
