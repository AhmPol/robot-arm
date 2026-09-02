#ifndef HELPERS_H
#define HELPERS_H

#include "raylib.h"
#include <math.h>
#include <stdio.h>

#define SCREEN_HEIGHT 650.0f
#define SCREEN_WIDTH 1000.0f
#define BALL_RADIUS 10.0f
#define ARM_LENGTH_SMALL 100.0f
#define ARM_LENGTH_BIG 200.0f
#define MAX_LENGTH (GetScreenWidth()/4 - 25)
#define MIN_LENGTH (ARM_LENGTH_BIG - ARM_LENGTH_SMALL)
#define MAX_SPEED 150.0f
#define ACCELERATION 150.0f
#define HALF_CIRCLE 180.0f
#define FULL_CIRCLE 360.0f
#define ANGLE_CORRECTION 0.1f
#define THICKNESS 10.0f
#define MAX_FRAME 60.0f
#define MAX_ERROR 600.0f

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

typedef struct Graph
{
    float x1;
    float x2;
    float y1;
    float y2;
} Graph;

typedef struct State
{
    bool angle1;
    bool angle2;
    bool target1;
    bool target2;
    bool speed1;
    bool speed2;
    bool error;
} State;

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
void recordState(bool *recording, bool *export, double *time, double *lastSample, int *sample);
void startMovementTime(Arm arm, double *startTime, bool *timing);
void stopMovementTime(float error, double *endTime, bool *timing, double startTime);
void recordTelemetry(double time, double *lastSample, int *sample, bool *recording, Arm arm, Telemetry data[], float error);
bool recordCSV(bool recording, int *sample, bool *export, Telemetry data[]);
void drawArm(Vector2 base, Vector2 target, Arm arm);
void drawGraphLayout(int length);
void drawTime(int i, Telemetry data[], int sample, Graph *graph);
void drawValue(float value1, float value2, float max, Graph *graph, Color color);
void drawMode (int i, Telemetry data[], Graph *graph, State state);
void drawGraphValues(int sample, Telemetry data[], Graph *graph, State state);
void drawLabels(double time, State state);

#endif