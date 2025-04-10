import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class multiThread {

    public static void main(String[] args) {
        UltraThread ultraThread = new UltraThread(); // Shared instance

        Thread motorThread = new Thread(new MotorThread(ultraThread)); // Pass it to MotorThread
        Thread ultraSensorThread = new Thread(ultraThread);

        ultraSensorThread.start();
        motorThread.start();
    }
}

class MotorThread implements Runnable  {
    private UltraThread ultraThread;

    public MotorThread(UltraThread ultraThread) {
        this.ultraThread = ultraThread;
    }

    public void run() {
        while (true) {
            float distance = ultraThread.getDistance(); // safe getter
            if (distance <= 0.5f) {
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                LCD.drawString("toimii", 0, 1);
            } else {
                Motor.A.forward();
                Motor.B.forward();
            }
            LCD.drawString("Tacho: " + Motor.A.getTachoCount(), 0, 2);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }      
    }
}

class UltraThread implements Runnable  {
    private float distance = Float.MAX_VALUE; // Default large distance

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (true) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0]; // Update shared variable

            //LCD.clear();
            //LCD.drawString("Distance: " + distance, 0, 0);

            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }     
    }
}
