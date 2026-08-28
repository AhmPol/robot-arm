#include "helpers.h"

void recordState(bool *recording, bool *export, double *time, double *lastSample)
{
    if (*recording == true) 
    {
        *recording = false;
    }
    else 
    {
        *recording = true;
        *time = GetTime();
        *lastSample = 0;
        *export = false;
    }
}

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
    float temp_distance = sqrtf(temp_d.x*temp_d.x + temp_d.y*temp_d.y);
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
    else if (temp_distance == 0)
    {
        d->x = 100;
        d->y = 0;
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
    float initialCos = (distance*distance + ARM_LENGTH_BIG*ARM_LENGTH_BIG - ARM_LENGTH_SMALL*ARM_LENGTH_SMALL)/ (2*ARM_LENGTH_BIG*distance);
    if (initialCos > 1) initialCos = 1;
    if (initialCos < -1) initialCos = -1;
    
    if (mode == 1)
    {
        
        arm->joint1.target = endPointAngle - acosf(initialCos) * RAD2DEG;
        arm->joint2.target = elbowAngle;
    }
    else
    {
        arm->joint1.target = endPointAngle + acosf(initialCos) * RAD2DEG;
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

void startMovementTime(Arm arm, double *startTime, bool *timing)
{
    if ((fabsf(arm.joint1.dif) >= ANGLECORRECTION || fabsf(arm.joint2.dif) >= ANGLECORRECTION) && *timing == false)
    {
        *startTime = GetTime();
        *timing = true;
    }
}
void stopMovementTime(float error, double *endTime, bool *timing, double startTime)
{
    if (error < ANGLECORRECTION && *timing == true)
    {
        *endTime = GetTime() - startTime;
        *timing = false;
    }
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

void getTelemetry(double time, Arm arm, Telemetry telemetry[], float error, int count)
{
    telemetry[count].target1 = arm.joint1.target;
    telemetry[count].target2 = arm.joint2.target;
    telemetry[count].angle1 = arm.joint1.angle;
    telemetry[count].angle2 = arm.joint2.angle;
    telemetry[count].speed1 = arm.joint1.speed;
    telemetry[count].speed2 = arm.joint2.speed;
    telemetry[count].error = error;
    telemetry[count].time = time; 
}

void recordTelemetry(double time, double *lastSample, int *sample, bool *recording, Arm arm, Telemetry data[], float error)
{
    if (((time - *lastSample) >= 0.05) && (*sample < 10000) && (*recording == true))
    {
        *lastSample += 0.05;
        getTelemetry(*lastSample, arm, data, error, *sample);
        (*sample)++;
    }   
    else if (*sample >= 10000) *recording = false;
}

bool printCSV(Telemetry data[], int sample)
{
    FILE *file = fopen("data.csv", "w");
    if (file == NULL)
    {
        return false;
    }

    fprintf(file, "time,angle1,angle2,target1,target2,speed1,speed2,error\n");

    for(int i = 0; i < sample; i++)
    fprintf(file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
        data[i].time, 
        data[i].angle1, 
        data[i].angle2, 
        data[i].target1, 
        data[i].target2, 
        data[i].speed1, 
        data[i].speed2, 
        data[i].error);

    fclose(file);

    return true;
}

bool recordCSV(bool recording, int *sample, bool *export, Telemetry data[])
{
    if ((recording == false) && (*sample != 0))
    {
        if (*export == false)
        {
            if (!printCSV(data, *sample))
            {
                return false;
            }
            *export = true;
            *sample = 0;
            return true;      
        }
    }
    return true;   
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