#include "helpers.h"

int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "MySimulator");
    
    Arm arm = {0};

    float distance;
    float dt;
    float endPointAngle;
    double startTime;
    double endTime;
    float error;
    bool timing = false;
    int mode = 1;
    float elbowAngle;

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
        if (IsCursorHidden()) ShowCursor();
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) target = GetMousePosition();
        dt = GetFrameTime();
        
        clampVector(&distance, &d, target, base);
        
        endPointAngle = atan2f(d.y, d.x) * RAD2DEG;
        elbowAngle = acosf(((distance*distance - ARM_LENGTH_SMALL*ARM_LENGTH_SMALL - ARM_LENGTH_BIG*ARM_LENGTH_BIG) / (2*ARM_LENGTH_BIG*ARM_LENGTH_SMALL))) * RAD2DEG;
    
        selectMode(&arm, endPointAngle, elbowAngle, mode, distance);      
        correctAngle(&arm);
        getShortestDistance(&arm);

        if ((fabsf(arm.joint1.dif) >= ANGLECORRECTION || fabsf(arm.joint2.dif) >= ANGLECORRECTION) && timing == false)
        {
            startTime = GetTime();
            timing = true;
        }
        
        moveArm(&arm, dt);
        correctAngle(&arm);
        getPosition(&arm, base);
        error = sqrt((d.x + base.x - arm.joint2.position.x)*(d.x + base.x - arm.joint2.position.x) + (d.y + base.y - arm.joint2.position.y)*(d.y + base.y - arm.joint2.position.y));

        if (error < ANGLECORRECTION && timing == true)
        {
            endTime = GetTime() - startTime;
            timing = false;
        }

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
            DrawText(TextFormat("Time: %.2f", endTime), THICKNESS, 120, 20, WHITE);          

        EndDrawing();
    }
    CloseWindow();
    return 0;
}