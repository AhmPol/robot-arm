#include "helpers.h"

Vector2 getTargetDelta(Vector2 target, Vector2 initial)
{
    Vector2 d;

    d.x = target.x - initial.x;
    d.y = target.y - initial.y;

    return d;
}

void clampVector(float *distance, Vector2 *d, Vector2 target, Vector2 base)
{
    Vector2 temp_d = getTargetDelta(target, base);
    float temp_distance = sqrt(temp_d.x*temp_d.x + temp_d.y*temp_d.y);
    if (temp_distance > MAX_LENGTH)
    {
        d->x = (temp_d.x/(temp_distance))*MAX_LENGTH;
        d->y = (temp_d.y/(temp_distance))*MAX_LENGTH;
        *distance = MAX_LENGTH;
    }
    else if (temp_distance < fabsf(MIN_LENGTH) && temp_distance > 0)
    {
        d->x = (temp_d.x/(temp_distance))*fabsf(MIN_LENGTH);
        d->y = (temp_d.y/(temp_distance))*fabsf(MIN_LENGTH);
        *distance = fabsf(MIN_LENGTH);
    }
    else
    {
        *d = temp_d;
        *distance = temp_distance;
    }
}

void selectMode(Arm *arm, float endPointAngle, float elbowAngle, int mode, float distance)
{
    if (mode == 1)
    {
        arm->joint1.target = endPointAngle - acosf((distance*distance + ARM_LENGTH_BIG*ARM_LENGTH_BIG - ARM_LENGTH_SMALL*ARM_LENGTH_SMALL)/ (2*ARM_LENGTH_BIG*distance)) * RAD2DEG;
        arm->joint2.target = elbowAngle;
    }
    else
    {
        arm->joint1.target = endPointAngle + acosf((distance*distance + ARM_LENGTH_BIG*ARM_LENGTH_BIG - ARM_LENGTH_SMALL*ARM_LENGTH_SMALL)/ (2*ARM_LENGTH_BIG*distance)) * RAD2DEG;
        arm->joint2.target = -elbowAngle;
    }
}

void getShortestDistanceHelper(Joint *joint)
{
    joint->dif = joint->target - joint->angle;

    if (joint->dif > HALFCIRCLE) joint->dif -= FULLCIRCLE;
    else if (joint->dif < - HALFCIRCLE) joint->dif += FULLCIRCLE;
}

void getShortestDistance(Arm *arm)
{
    getShortestDistanceHelper(&arm->joint1);
    getShortestDistanceHelper(&arm->joint2);
}

void correctAngleHelper(Joint *joint)
{
    if (joint->target > HALFCIRCLE) joint->target -= FULLCIRCLE;
    else if (joint->target < -HALFCIRCLE) joint->target += FULLCIRCLE;

    if (joint->angle > HALFCIRCLE) joint->angle -= FULLCIRCLE;
    else if (joint->angle < -HALFCIRCLE) joint->angle += FULLCIRCLE;
}

void correctAngle(Arm *arm)
{
    correctAngleHelper(&arm->joint1);
    correctAngleHelper(&arm->joint2);
}

void moveArmHelper(Joint *joint, float dt)
{
    if ((fabsf(joint->dif) < ANGLECORRECTION)) 
    {
        joint->angle = joint->target;
        joint->speed = 0;
    }
    else
    {
        float stopAngle = ((joint->speed)*(joint->speed))/(2*ACCELERATION);

        if (fabsf(joint->dif) <= stopAngle) joint->speed -= ACCELERATION * dt;
        else joint->speed += ACCELERATION * dt;

        if (joint->speed > MAX_SPEED) joint->speed = MAX_SPEED;
        if (joint->speed < 0) joint->speed = 0;

        if (joint->dif > 0.0f) joint->angle += joint->speed * dt;
        else if (joint->dif < 0.0f) joint->angle -= joint->speed * dt;
    }
}

void moveArm(Arm *arm, float dt)
{
    moveArmHelper(&arm->joint1, dt);
    moveArmHelper(&arm->joint2, dt);
}

void getPositionHelper(Joint *joint, Vector2 initial, float firstAngle,float secondAngle, float length)
{  
    joint->position.x = initial.x + length * cosf((firstAngle + secondAngle) * DEG2RAD);
    joint->position.y = initial.y + length * sinf((firstAngle + secondAngle) * DEG2RAD);
}

void getPosition(Arm *arm, Vector2 base)
{
    getPositionHelper(&arm->joint1, base, arm->joint1.angle, 0.0f, ARM_LENGTH_BIG);
    getPositionHelper(&arm->joint2, arm->joint1.position, arm->joint1.angle, arm->joint2.angle, ARM_LENGTH_SMALL);
}

void drawLines(Vector2 base, Vector2 joint1, Vector2 joint2)
{
    DrawLineEx(base, joint1, THICKNESS, GRAY);
    DrawLineEx(joint1, joint2, THICKNESS, GRAY);
    // DrawLineV(base, joint2, RED);
}

void drawCircles(Vector2 base, Vector2 joint1, Vector2 joint2, Vector2 target)
{
    DrawCircleV(base, (float)BALL_RADIUS, MAROON);
    DrawCircleV(joint1, (float)BALL_RADIUS, GREEN);
    DrawCircleV(target, (float)BALL_RADIUS, BLUE);
    // DrawCircleLinesV(base, armLength*2, WHITE);
    DrawCircleV(joint2, (float)BALL_RADIUS, PURPLE);
}