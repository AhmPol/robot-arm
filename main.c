#include "helpers.h"

int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "MySimulator");
    
    Arm arm = {0};

    float distance = 0.0;
    float dt = 0.0;
    float endPointAngle = 0.0;
    float error = 0.0;
    float elbowAngle = 0.0;
    
    int mode = 1;
    double startTime = 0.0;
    double endTime = 0.0;
    double lastSample = 0.0; 
    double recordStart = 0.0;
    double time = 0.0;
    int sample = 0;
    
    bool timing = false;
    bool recording = false;
    bool export = false;
    
    Telemetry data[10000];

    const Vector2 base = { GetScreenWidth()/2.0f, GetScreenHeight()/2.0f};
    Vector2 target = {base.x + MAX_LENGTH, base.y};
    Vector2 d;

    SetTargetFPS(MAX_FRAME);    
    HideCursor();

    while(!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_H))
        {
            if (mode == 1) mode = 2;
            else mode = 1;
        }
        if (IsKeyPressed(KEY_R)) recordState(&recording, &export, &recordStart, &lastSample);

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

        if (!recordCSV(recording, &sample, &export, data)) return 1;

        BeginDrawing();
            ClearBackground(BLACK);

            drawLines(base, arm.joint1.position, arm.joint2.position);
            drawCircles(base, arm.joint1.position, arm.joint2.position, target);
            DrawText(TextFormat("Angle1: %.2f", arm.joint1.angle), THICKNESS, 0, 20, WHITE);         
            DrawText(TextFormat("Angle2: %.2f", arm.joint2.angle), THICKNESS, 20, 20, WHITE);         
            DrawText(TextFormat("Target1: %.2f", arm.joint1.target), THICKNESS, 40, 20, WHITE);         
            DrawText(TextFormat("Target2: %.2f", arm.joint2.target), THICKNESS, 60, 20, WHITE);         
            DrawText(TextFormat("Endpoint: %.2f", endPointAngle), THICKNESS, 80, 20, WHITE);          
            DrawText(TextFormat("Mode: %i", mode), THICKNESS, 100, 20, WHITE);          
            DrawText(TextFormat("Robot Taken: %.2fs", endTime), THICKNESS, 120, 20, WHITE);          
            DrawText(TextFormat("Current Time: %.2fs", time), THICKNESS, 140, 20, WHITE);          
            DrawText(TextFormat("Record Start: %.2fs", recordStart), THICKNESS, 160, 20, WHITE);          
            DrawText(TextFormat("Last Sample: %.2fs", lastSample), THICKNESS, 180, 20, WHITE);          
            DrawText(TextFormat("Sample Count: %i", sample), THICKNESS, 200, 20, WHITE);          

        EndDrawing();
    }
    CloseWindow();
    return 0;
}