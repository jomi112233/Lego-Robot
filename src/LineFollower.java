import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    private static final float KP = 1000.0f;  // Proportional gain for line-following
    private static final float KI = 0.0f;    // Integral gain (currently unused)
    private static final float KD = 100.0f;  // Derivative gain for line-following
    private static final int BASE_SPEED = 225;
    private static final int MAX_SPEED = 300;
    private static final float OBSTACLE_DISTANCE = 0.5f; // Minimum distance for obstacle detection

    private static float integral = 0;
    private static float lastError = 0;
    private static float distance = Float.MAX_VALUE;

    public static void main(String[] args) {
        LCD.drawString("Press any button", 0, 0);
        LCD.drawString("to start", 0, 1);
        Button.waitForAnyPress();
        LCD.clear();

        try {
            // Initialize the sensors
            EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
            EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
            SampleProvider colorProvider = colorSensor.getRedMode();
            SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
            float[] colorSample = new float[colorProvider.sampleSize()];
            float[] distanceSample = new float[distanceMode.sampleSize()];

            // Calibration phase
            LCD.drawString("Calibrating...", 0, 0);
            LCD.drawString("Place on line", 0, 1);
            Button.waitForAnyPress();
            colorProvider.fetchSample(colorSample, 0);
            float lineValue = colorSample[0];

            LCD.drawString("Place off line", 0, 0);
            Button.waitForAnyPress();
            colorProvider.fetchSample(colorSample, 0);
            float backgroundValue = colorSample[0];

            float targetValue = (lineValue + backgroundValue) / 2;

            LCD.clear();
            LCD.drawString("Following line", 0, 0);

            while (!Button.ESCAPE.isDown()) {
                // Update distance reading from ultrasonic sensor
                distanceMode.fetchSample(distanceSample, 0);
                distance = distanceSample[0];

                // Obstacle avoidance logic
                if (distance <= OBSTACLE_DISTANCE) {
                    // If an obstacle is detected, rotate to avoid it
                    Motor.A.rotate(-360, true);  // Rotate Motor A backward
                    Motor.B.rotate(360);         // Rotate Motor B forward
                    Delay.msDelay(1000);         // Wait for a moment
                    continue;                    // Skip the rest of the loop and check again
                }

                // Line following logic
                colorProvider.fetchSample(colorSample, 0);
                float currentValue = colorSample[0];
                float error = targetValue - currentValue;

                // PID control for smoother line-following
                integral = integral * 0.5f + error;
                float derivative = error - lastError;
                float correction = KP * error + KI * integral + KD * derivative;

                int speedA = (int)(BASE_SPEED + correction);
                int speedB = (int)(BASE_SPEED - correction);

                speedA = Math.min(Math.max(speedA, -MAX_SPEED), MAX_SPEED);
                speedB = Math.min(Math.max(speedB, -MAX_SPEED), MAX_SPEED);

                // Set motor speeds based on calculated correction
                Motor.A.setSpeed(Math.abs(speedA));
                Motor.B.setSpeed(Math.abs(speedB));

                if (speedA > 0) {
                    Motor.A.forward();
                } else {
                    Motor.A.backward();
                }

                if (speedB > 0) {
                    Motor.B.forward();
                } else {
                    Motor.B.backward();
                }

                // Update last error for derivative calculation in next loop
                lastError = error;
                Delay.msDelay(10);  // Short delay to allow motors to adjust
            }

            // Stop motors and close sensors when ESCAPE button is pressed
            Motor.A.stop();
            Motor.B.stop();
            colorSensor.close();
            ultrasonicSensor.close();

            LCD.clear();
            LCD.drawString("Done!", 0, 0);

        } catch (Exception e) {
            LCD.clear();
            LCD.drawString("Error", 0, 0);
            LCD.drawString(e.getMessage(), 0, 1);
        }

        Button.waitForAnyPress();
    }
}
