package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp(name="test dashboard 3", group="Linear Opmode")

public class TestDashboardTois extends LinearOpMode {

    private static final String VUFORIA_KEY =
            " Ac0FFOT/////AAABmXCneE7i4E7yh04pdQJzJLoJiMTtdAAblmjvED9fhNmHt4e0yFN4TffUBMeonbsrrhvdOrdOmUsxVQxe0hchzvvmeqSXKhR3Ozn6e9tiwJ7tJv2DCpPVgCPohRGMT/ZVdUi+IXnFA2tC1O+xfHJ/0Mtm6V4xq/WHNeP1IeIOTVewRYf9XtZBhkpS+Fa+SdCsnalhvv1sF2Ss+wz9m6cUKo6KoEz80ItsN2UUTlxMsnOQaAXfELJ/zeTCk7Nl4u8T5E+aoeLjI31HXUSYzy/JsT9dUUXCPDz95KHLWm2nxRPlciqLik6ZLJKCMYzdn37t8CjYkZxKmrvCFMgjrkErM4a/UtCLqM3iyr8Sb/fnSOdW ";

    WebcamName webcamName = null;

    // Declare OpMode members.
    DcMotorEx x, y, y2;

    // encoder position tracking parameters

    final int TICKS_PER_REV = 8192;
    final int RADIUS = 30; // mm
    final double DISTANCE_PER_REV = RADIUS * 2.0 * Math.PI;
    final double DISTANCE_BETWEEN_AXIS = 415;
    final double CIRCUM = DISTANCE_BETWEEN_AXIS * Math.PI;
    final double SIDE_LENGTH = 10;


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        x = hardwareMap.get(DcMotorEx.class,"x");
        y = hardwareMap.get(DcMotorEx.class,"y");
        y2 = hardwareMap.get(DcMotorEx.class,"y2");
        double xDistance, yTurn, rotationAngle, yTurn2;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        boolean isTurning = false;

        // run until the end of the match (driver presses STOP)
        double centerX = 0;
        double centerY = 0;

        double deltaX = 0;
        double deltaY = 0;

        double prevX = 0;
        double prevY = 0;

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            xDistance = (double) x.getCurrentPosition() / TICKS_PER_REV * DISTANCE_PER_REV;
            yTurn = (double) y.getCurrentPosition() / TICKS_PER_REV * DISTANCE_PER_REV;
            yTurn2 = -(double) y2.getCurrentPosition() / TICKS_PER_REV * DISTANCE_PER_REV;
            double yDistance = ((double) y.getCurrentPosition() / TICKS_PER_REV * DISTANCE_PER_REV - (double) y2.getCurrentPosition() / TICKS_PER_REV * DISTANCE_PER_REV) / 2;

            rotationAngle = (yTurn2 - yTurn) / (2* CIRCUM);
            rotationAngle = rotationAngle * 360 * Math.PI / 180;

            deltaX = xDistance - prevX;
            deltaY = yDistance - prevY;

            packet.put("Angle", rotationAngle);

            centerX += Math.cos(rotationAngle) * (deltaX) + Math.sin(rotationAngle) * (deltaY);
            centerY += Math.sin(rotationAngle) * (deltaX) + Math.cos(rotationAngle) * (deltaY);
            double sin = Math.sin(rotationAngle);
            double cos = Math.cos(rotationAngle);

            double[] pointX = {-SIDE_LENGTH * cos - SIDE_LENGTH * sin + centerX,
                               +SIDE_LENGTH * cos - SIDE_LENGTH * sin + centerX,
                               +SIDE_LENGTH * cos - -SIDE_LENGTH * sin + centerX,
                               -SIDE_LENGTH * cos - -SIDE_LENGTH * sin + centerX};

            double[] pointY = {-SIDE_LENGTH * sin + SIDE_LENGTH * cos + centerY,
                               +SIDE_LENGTH * sin + SIDE_LENGTH * cos + centerY,
                               +SIDE_LENGTH * sin + -SIDE_LENGTH * cos + centerY,
                               -SIDE_LENGTH * sin + -SIDE_LENGTH * cos + centerY};

            packet.put("Cx", centerX);
            packet.put("Cy", centerY);
            packet.put("pointX", pointX[0]);
            packet.put("pointY", pointY[0]);

            prevX = xDistance; prevY = yDistance;

            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .strokeCircle(centerX, centerY, 10);

            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .strokePolygon(pointX, pointY);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
