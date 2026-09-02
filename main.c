#include "helpers.h"

int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1000, 650, "MySimulator");
    MaximizeWindow();

    Arm arm = {0};
    Graph graph = {0};
    Telemetry data[10000];
    State state = {false};

    float distance = 0.0;
    float dt = 0.0;
    float endPointAngle = 0.0;
    float error = 0.0;
    float elbowAngle = 0.0;

    int length = GetScreenWidth()/2 - 40;
    int teleMode = 0;
    int sample = 0;

    double startTime = 0.0;
    double endTime = 0.0;
    double lastSample = 0.0; 
    double recordStart = 0.0;
    double time = 0.0;
    
    bool timing = false;
    bool recording = false;
    bool export = false;
    bool mode = false;

    Vector2 base = { GetScreenWidth()/4.0f, GetScreenHeight()/2.0f};
    Vector2 target = {base.x + MAX_LENGTH, base.y};
    Vector2 d;

    SetTargetFPS(MAX_FRAME);    
    HideCursor();

    while(!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_H)) mode = !mode;
        if (IsKeyPressed(KEY_R)) recordState(&recording, &export, &recordStart, &lastSample, &sample);
        if (IsKeyPressed(KEY_T)) state.angle1 = !state.angle1;
        if (IsKeyPressed(KEY_Y)) state.angle2 = !state.angle2;
        if (IsKeyPressed(KEY_U)) state.target1 = !state.target1;
        if (IsKeyPressed(KEY_I)) state.target2 = !state.target2;
        if (IsKeyPressed(KEY_O)) state.speed1 = !state.speed1;
        if (IsKeyPressed(KEY_P)) state.speed2 = !state.speed2;
        if (IsKeyPressed(KEY_L)) state.error = !state.error;

        if (IsCursorHidden()) ShowCursor();
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) target = GetMousePosition();
        dt = GetFrameTime();
        time = GetTime() - recordStart;
        
        clampVector(&distance, &d, target, base);
        
        endPointAngle = atan2f(d.y, d.x) * RAD2DEG;
        elbowAngle = (distance*distance - ARM_LENGTH_SMALL*ARM_LENGTH_SMALL - ARM_LENGTH_BIG*ARM_LENGTH_BIG) / (2*ARM_LENGTH_BIG*ARM_LENGTH_SMALL);
        if (elbowAngle > 1) elbowAngle = 1;
        if (elbowAngle < -1) elbowAngle = -1;
        elbowAngle = acosf(elbowAngle) * RAD2DEG;
    
        selectMode(&arm, endPointAngle, elbowAngle, mode, distance);      
        correctAngle(&arm);
        getShortestDistance(&arm);

        startMovementTime(arm, &startTime, &timing);
        
        moveArm(&arm, dt);
        correctAngle(&arm);
        getPosition(&arm, base);
        error = sqrt((d.x + base.x - arm.joint2.position.x)*(d.x + base.x - arm.joint2.position.x) + (d.y + base.y - arm.joint2.position.y)*(d.y + base.y - arm.joint2.position.y));

        stopMovementTime(error, &endTime, &timing, startTime);

        recordTelemetry(time, &lastSample, &sample, &recording, arm, data, error);

        BeginDrawing();
            ClearBackground(BLACK);
            drawArm(base, target, arm);
            drawGraphLayout(length);
            drawGraphValues(sample, data, &graph, state);
            drawLabels(time, state);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}