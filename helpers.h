#ifndef HELPERS_H
#define HELPERS_H

#include "raylib.h"
#include <math.h>
#include <stdio.h>

#define SCREEN_HEIGHT 650
#define SCREEN_WIDTH 1000
#define BALL_RADIUS 10
#define ARM_LENGTH_SMALL 100.0f
#define ARM_LENGTH_BIG 200.0f
#define MAX_LENGTH (ARM_LENGTH_BIG + ARM_LENGTH_SMALL)
#define MIN_LENGTH (ARM_LENGTH_BIG - ARM_LENGTH_SMALL)
#define MAX_SPEED 150.0f
#define ACCELERATION 150.0f
#define HALFCIRCLE 180.0f
#define FULLCIRCLE 360.0f
#define ANGLECORRECTION 0.1f
#define THICKNESS 10.0f
#define MAX_FRAME 60

typedef struct Joint
{
    float angle;
    float target;
    float dif;
    float speed;  
    Vector2 position;
} Joint;

typedef struct Arm
{
    Joint joint1;
    Joint joint2;
} Arm;

typedef struct Telemetry
{
    double time;
    float angle1;
    float angle2;
    float target1;
    float target2;
    float speed1;
    float speed2;
    float error;
} Telemetry;

Vector2 getTargetDelta(Vector2 target, Vector2 initial);
void correctAngle(Arm *arm);
void getShortestDistance(Arm *arm);
void moveArm(Arm *arm, float dt);
void getPosition(Arm *arm, Vector2 base);
void drawLines(Vector2 base, Vector2 joint1, Vector2 joint2);
void drawCircles(Vector2 base, Vector2 joint1, Vector2 joint2, Vector2 target);
void selectMode(Arm *arm, float endpointAngle, float elbowAngle, int mode, float distance);
void clampVector(float *distance, Vector2 *d, Vector2 target, Vector2 base);
void getTelemetry(double time, Arm arm, Telemetry telemetry[], float error, int count);
bool printCSV(Telemetry data[], int sample);
void recordState(bool *recording, bool *export, double *time, double *lastSample);
void startMovementTime(Arm arm, double *startTime, bool *timing);
void stopMovementTime(float error, double *endTime, bool *timing, double startTime);
void recordTelemetry(double time, double *lastSample, int *sample, bool *recording, Arm arm, Telemetry data[], float error);
bool recordCSV(bool recording, int *sample, bool *export, Telemetry data[]);

#endif