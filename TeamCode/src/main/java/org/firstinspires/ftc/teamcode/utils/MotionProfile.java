package org.firstinspires.ftc.teamcode.utils;

public class MotionProfile implements MotionProfileInterface{
    private double INITIAL_POSITION;
    private double END_POSITION;

    private double MAX_VELOCITY;
    private double MAX_ACCELERATION;
    private double MAX_DECELERATION;

    private double ACCELERATION_TIME;
    private double DECELERATION_TIME;
    private double MAX_VELOCITY_TIME;

    private double MOTION_PROFILE_CREATION_TIME;
    private boolean isReversed;

    public MotionProfile(double INITIAL_POSITION, double END_POSITION, double MAX_ACCELERATION, double MAX_VELOCITY, double MAX_DECELERATION){
        this.INITIAL_POSITION = INITIAL_POSITION;
        this.END_POSITION = END_POSITION;
        this.MAX_VELOCITY = MAX_VELOCITY;
        this.MAX_ACCELERATION = MAX_ACCELERATION;
        this.MAX_DECELERATION = MAX_DECELERATION;

        double deltaX = END_POSITION - INITIAL_POSITION;

        if(deltaX < 0){
            deltaX = -deltaX;
            isReversed = true;
        }
        else {
            isReversed = false;
        }

        double AllAccelerationTime = Math.min((MAX_ACCELERATION + MAX_DECELERATION) * MAX_VELOCITY / (MAX_ACCELERATION * MAX_DECELERATION),
                Math.sqrt((MAX_ACCELERATION + MAX_DECELERATION) * 2.0 * deltaX / (MAX_ACCELERATION * MAX_DECELERATION)));
        ACCELERATION_TIME = (AllAccelerationTime * MAX_DECELERATION) / (MAX_ACCELERATION + MAX_ACCELERATION);
        DECELERATION_TIME = (AllAccelerationTime * MAX_ACCELERATION) / (MAX_ACCELERATION + MAX_ACCELERATION);
        MAX_VELOCITY_TIME = Math.max(0, deltaX - MAX_ACCELERATION * ACCELERATION_TIME * ACCELERATION_TIME / 2.0 - MAX_DECELERATION * DECELERATION_TIME * DECELERATION_TIME / 2.0 );

        MOTION_PROFILE_CREATION_TIME = System.nanoTime() /  1e9;

    }


    @Override
    public double getPosition() {
        double deltaTime = System.nanoTime()/1e9 - MOTION_PROFILE_CREATION_TIME;
        if(deltaTime <= ACCELERATION_TIME) {
            return INITIAL_POSITION + MAX_ACCELERATION * deltaTime * deltaTime / 2 * (isReversed ? -1 : 1);
        }
        else if(deltaTime <= ACCELERATION_TIME + MAX_VELOCITY_TIME){
            return INITIAL_POSITION + ((MAX_ACCELERATION * ACCELERATION_TIME * ACCELERATION_TIME / 2) + MAX_ACCELERATION * ACCELERATION_TIME * (deltaTime - ACCELERATION_TIME) / 2) * (isReversed ? -1 : 1);
        }
        else if(deltaTime<=ACCELERATION_TIME + MAX_VELOCITY_TIME + DECELERATION_TIME){
            return INITIAL_POSITION + ((MAX_ACCELERATION * ACCELERATION_TIME * ACCELERATION_TIME / 2) + MAX_ACCELERATION * ACCELERATION_TIME *
                    (deltaTime - ACCELERATION_TIME - MAX_VELOCITY_TIME) / 2 - MAX_DECELERATION * DECELERATION_TIME * (deltaTime - ACCELERATION_TIME - DECELERATION_TIME) / 2.0) * (isReversed ? -1 : 1);
        }
        else
            return END_POSITION;
    }

    @Override
    public double getVelocity() {
        double deltaTime = System.nanoTime() / 1e9 - MOTION_PROFILE_CREATION_TIME;
        if(deltaTime <= ACCELERATION_TIME){
            return MAX_ACCELERATION * deltaTime * (isReversed ? -1 : 1);
        }
        else if (deltaTime <= ACCELERATION_TIME + MAX_VELOCITY_TIME){
            return MAX_VELOCITY * (isReversed ? -1 : 1);
        }
        else if (deltaTime <= ACCELERATION_TIME + MAX_VELOCITY_TIME + DECELERATION_TIME){
            return (-MAX_DECELERATION * (deltaTime - ACCELERATION_TIME - MAX_VELOCITY_TIME) + MAX_ACCELERATION * ACCELERATION_TIME) * (isReversed ? -1 : 1);
        }
        else
            return 0;
    }

    @Override
    public double getAcceleration() {
        double deltaTime = System.nanoTime()/1e9 - MOTION_PROFILE_CREATION_TIME;
        if(deltaTime <= ACCELERATION_TIME){
            return MAX_ACCELERATION * (isReversed ? -1 : 1);
        }
        else if(ACCELERATION_TIME + MAX_VELOCITY_TIME <= deltaTime && deltaTime <= ACCELERATION_TIME + MAX_VELOCITY_TIME + DECELERATION_TIME){
            return MAX_DECELERATION * (isReversed ? -1 : 1);
        }
        else
            return 0;
    }

    @Override
    public boolean isFinished() {
        double deltaTime = System.nanoTime() / 1e9 - MOTION_PROFILE_CREATION_TIME;
        return deltaTime > ACCELERATION_TIME + MAX_VELOCITY_TIME + DECELERATION_TIME;
    }
}