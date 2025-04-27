import lejos.hardware.motor.Motor;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class BBLineFollower {
    public static void main(String[] args) {
        LightSensor lightData = new LightSensor();
        UltraSensor ultradata = new UltraSensor();
        calculator calculator = new calculator(lightData, ultradata);

        Thread lightThread = new Thread(lightData);
        Thread ultraThread = new Thread(ultradata);
        Thread calculatorThread = new Thread(calculator);
        Thread motorThread = new Thread(new Motors(calculator));

        lightThread.start();
        ultraThread.start();
        calculatorThread.start();
        motorThread.start();
    }
}

class calculator implements Runnable {
    private static final float KP = 80.0f;
    private static final float KI = 1.0f;
    private static final float KD = 20.0f;
    private static final int BASE_SPEED = 200;
    private static final int MAX_SPEED = 300;
    private static final float OBSTACLE_DISTANCE = 0.3f;

    private static float integral = 0;
    private static float lastError = 0;

    private enum AvoidState { IDLE, TURN_AWAY, GO_FORWARD, TURN_BACK, FIND_LINE }
    private AvoidState avoidState = AvoidState.IDLE;
    private long stateStartTime = 0;

    private boolean isAvoiding = false;

    private int TrueSpeedA;
    private int TrueSpeedB;

    private LightSensor lightSensor;
    private UltraSensor ultraSensor;

    public calculator(LightSensor lightSensor, UltraSensor ultraSensor) {
        this.lightSensor = lightSensor;
        this.ultraSensor = ultraSensor;
    }

    public void run() {
        while (true) {
            float colorSample = lightSensor.GetLight();
            float distance = ultraSensor.getDistance();

            if (distance <= OBSTACLE_DISTANCE && !isAvoiding) {
                isAvoiding = true;
                transitionTo(AvoidState.TURN_AWAY);
            }

            if (isAvoiding) {
                goAround(distance);
            } else {
                drive(colorSample);
            }

            try {
                Thread.sleep(10); // prevent busy loop
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void drive(float colorSample) {
        float backgroundValue = 0.8f;
        float lineValue = 0.2f;
        float targetValue = (lineValue + backgroundValue) / 2;

        float error = targetValue - colorSample;

        integral = integral * 0.5f + error;
        float derivative = error - lastError;
        float correction = KP * error + KI * integral + KD * derivative;

        correction = Math.max(Math.min(correction, 300), -300);

        int speedA = (int) (BASE_SPEED + correction);
        int speedB = (int) (BASE_SPEED - correction);

        speedA = Math.min(Math.max(speedA, -MAX_SPEED), MAX_SPEED);
        speedB = Math.min(Math.max(speedB, -MAX_SPEED), MAX_SPEED);

        TrueSpeedA = speedA;
        TrueSpeedB = speedB;

        lastError = error;
    }

    private void goAround(float dist) {
        long now = System.currentTimeMillis();
        long elapsed = now - stateStartTime;

        switch (avoidState) {
            case TURN_AWAY:
                TrueSpeedA = 100;
                TrueSpeedB = -100;
                if (elapsed > 1000) {
                    transitionTo(AvoidState.GO_FORWARD);
                }
                break;

            case GO_FORWARD:
                TrueSpeedA = 150;
                TrueSpeedB = 150;
                if (elapsed > 1500) {
                    transitionTo(AvoidState.TURN_BACK);
                }
                break;

            case TURN_BACK:
                TrueSpeedA = -100;
                TrueSpeedB = 100;
                if (elapsed > 1000) {
                    transitionTo(AvoidState.FIND_LINE);
                }
                break;

            case FIND_LINE:
                TrueSpeedA = 100;
                TrueSpeedB = 100;
                if (lightSensor.GetLight() < 0.5f) {
                    isAvoiding = false;
                    avoidState = AvoidState.IDLE;
                }
                break;

            default:
                break;
        }

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void transitionTo(AvoidState newState) {
        avoidState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    public int GetSpeedA() {
        return TrueSpeedA;
    }

    public int GetSpeedB() {
        return TrueSpeedB;
    }
}

class Motors implements Runnable {
    private calculator Calculator;

    public Motors(calculator Calculator) {
        this.Calculator = Calculator;
    }

    public void run() {
        int speedA;
        int speedB;

        while (true) {
            speedA = Calculator.GetSpeedA();
            speedB = Calculator.GetSpeedB();

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

            try {
                Thread.sleep(10); // keep it light
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

class UltraSensor implements Runnable {
    private volatile float distance = Float.MAX_VALUE;

    EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
    SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
    float[] distanceSample = new float[distanceMode.sampleSize()];

    public float getDistance() {
        return distance;
    }

    public void run() {
        while (true) {
            distanceMode.fetchSample(distanceSample, 0);
            distance = distanceSample[0];

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

class LightSensor implements Runnable {
    private volatile float Light = Float.MAX_VALUE;

    EV3ColorSensor LightSensor = new EV3ColorSensor(SensorPort.S2);
    SampleProvider LightProvider = LightSensor.getRedMode(); // try getRedMode for line following
    float[] LightSample = new float[LightProvider.sampleSize()];

    public float GetLight() {
        return Light;
    }

    public void run() {
        while (true) {
            LightProvider.fetchSample(LightSample, 0);
            Light = LightSample[0];

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
