/* Copyright (c) 2017 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name="VuMark Id One Shot", group ="Concept")
//@Disabled
public class VuMarkIdOneShot extends LinearVisionOpMode {
  private Runnerbot robot = new Runnerbot (this);
  private String targetJewel = "";

  @Override
  public void runOpMode() throws InterruptedException {
    VuforiaTrackable relicTemplate = robot.initVuforia("RelicVuMark");
    RelicRecoveryVuMark vuMark;

    waitForStart();
    do {
      /**
       * See if any of the instances of {@link relicTemplate} are currently visible.
       * {@link RelicRecoveryVuMark} is an enum which can have the following values:
       * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
       * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
       */
      vuMark = RelicRecoveryVuMark.from(relicTemplate);
      if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
        /* Found an instance of the template. In the actual game, you will probably
         * loop until this condition occurs, then move on to act accordingly depending
         * on which VuMark was visible. */
        telemetry.addData("VuMark", "%s visible", vuMark);
      }
      else {
        telemetry.addData("VuMark", "not visible");
      }

      telemetry.update();
    } while (vuMark == RelicRecoveryVuMark.UNKNOWN);
  }
}
