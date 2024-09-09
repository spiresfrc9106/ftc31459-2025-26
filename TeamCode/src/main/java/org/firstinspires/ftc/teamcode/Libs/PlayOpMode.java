package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libs.Classes.GameController;

public abstract class PlayOpMode extends LinearOpMode{
    protected GameController playerController, operatorController;
    ElapsedTime timer;
    protected boolean isTeleOp;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        preInitialize(); // Run pre-initialization code, if any
        if (isTeleOp) {
            playerController = new GameController(gamepad1);
            operatorController = new GameController(gamepad2);
        }
        initialize(); // Initialize the hardware

        if (isTeleOp) {

            playerController.updateButtonStates();
            operatorController.updateButtonStates();

        }
        waitForStart(); // Wait for start button

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double dt = timer.milliseconds() / 1000.0; // Delta time in seconds
                if (isTeleOp) {
                    playerController.updateButtonStates();
                    operatorController.updateButtonStates();
                }
                run(dt); // Loop

//                sleep(0); // Do not waste CPU cycle but we dont want any lag either
                timer.reset();
                sleep(100);
            }
        }
    }

    /**
     * Override this method to run pre-initialization code.
     * Like setting isTeleOp to true
     */
    protected void preInitialize() {
        isTeleOp = false;
    }

    /**
     * Initialize
     */
    protected abstract void initialize();

    /**
     * Loop
     */
    protected abstract void run(double dt) throws InterruptedException;
}
