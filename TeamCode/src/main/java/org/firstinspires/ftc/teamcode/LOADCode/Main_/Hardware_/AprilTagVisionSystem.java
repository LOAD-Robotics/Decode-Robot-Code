/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagVisionSystem {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detections;
    private OpMode opMode;

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag(OpMode opmode) {
        opMode = opmode;

        // Create the AprilTag processor.
        tagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(tagProcessor, true);
    }
    /**
     * Add telemetry about AprilTag detections.
     */
    public void printDebugTelemetry() {
        updateAprilTagProcessor();
        opMode.telemetry.addData("# AprilTags Detected", detections.size());

        // Step through the list of detections and display info for each one.
        for (int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", getID(i), getMetadata(i).name));
                opMode.telemetry.addLine(getXYZ(i).toString());
                opMode.telemetry.addLine(getPRYbyIndex(i).toString());
                opMode.telemetry.addLine(getRBE(i).toString());
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", getID(i)));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to opMode.telemetry
        opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        opMode.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    /**
     * @return a boolean representing whether a tag is detected by the camera
     */
    public boolean tagDetected(){
        return !detections.isEmpty();
    }
    public boolean tagDetected(int id){
        for (int i = 0; i < detections.size(); i++){
            if (detections.get(i).id == id){
                return true;
            }
        }
        return false;
    }

    /**
     * Fetches the most recent AprilTag detections
     */
    public void updateAprilTagProcessor(){
        detections = tagProcessor.getDetections();
    }

    public PoseXYZ getXYZbyIndex(int detectionIndex){
        updateAprilTagProcessor();
        return new PoseXYZ(detections.get(detectionIndex).ftcPose);
    }
    public PoseXYZ getXYZ(int id){
        for (int i = 0; i < detections.size(); i++){
            if (detections.get(i).id == id){
                return getXYZbyIndex(i);
            }
        }
        return null;
    }

    public PosePRY getPRYbyIndex(int detectionIndex){
        updateAprilTagProcessor();
        return new PosePRY(detections.get(detectionIndex).ftcPose);
    }
    public PosePRY getPRY(int id){
        for (int i = 0; i < detections.size(); i++){
            if (detections.get(i).id == id){
                return getPRYbyIndex(i);
            }
        }
        return null;
    }

    public PoseRBE getRBEbyIndex(int detectionIndex){
        updateAprilTagProcessor();
        return new PoseRBE(detections.get(detectionIndex).ftcPose);
    }
    public PoseRBE getRBE(int id){
        for (int i = 0; i < detections.size(); i++){
            if (detections.get(i).id == id){
                return getRBEbyIndex(i);
            }
        }
        return null;
    }

    public AprilTagMetadata getMetadata(int detectionIndex){
        updateAprilTagProcessor();
        return detections.get(detectionIndex).metadata;
    }
    public AprilTagMetadata getMetadata(){
        return getMetadata(0);
    }

    public int getID(int detectionIndex){
        return detections.get(detectionIndex).id;
    }
    public int getID(){
        return getID(0);
    }

    public class PoseXYZ {
        public double x;
        public double y;
        public double z;

        PoseXYZ(double X, double Y, double Z){
            x = X;
            y = Y;
            z = Z;
        }
        PoseXYZ(AprilTagPoseFtc pose){
            x = pose.x;
            y = pose.y;
            z = pose.z;
        }
        @NonNull
        @Override
        public String toString(){
            return "[X: " + x + ", Y: " + y + ", Z: " + z + "]";
        }
    }
    public class PosePRY {
        public double p;
        public double r;
        public double y;

        PosePRY(double P, double R, double Y){
            p = P;
            r = R;
            y = Y;
        }
        PosePRY(AprilTagPoseFtc pose){
            p = pose.pitch;
            r = pose.roll;
            y = pose.yaw;
        }
        @NonNull
        @Override
        public String toString(){
            return "[Pitch: " + p + ", Roll: " + r + ", Yaw: " + y + "]";
        }
    }
    public class PoseRBE {
        public final double r;
        public final double b;
        public final double e;

        PoseRBE(double range, double bearing, double elevation){
            r = range;
            b = bearing;
            e = elevation;
        }
        PoseRBE(AprilTagPoseFtc pose){
            r = pose.range;
            b = pose.bearing;
            e = pose.elevation;
        }
        @NonNull
        @Override
        public String toString(){
            return "[Range: " + r + ",\n Bearing: " + b + ",\n Elevation: " + e + "]";
        }
    }
}
