#include "randice.h"

#include <vector>

#define FPS 60
#define CUBES 1

const Color COLORS[25] = {
    LIGHTGRAY, GRAY, DARKGRAY,
    YELLOW, GOLD, ORANGE,
    PINK, RED, MAROON,
    GREEN, LIME, DARKGREEN,
    SKYBLUE, BLUE, DARKBLUE,
    PURPLE, VIOLET, DARKPURPLE,
    BEIGE, BROWN, DARKBROWN,
    WHITE, BLACK, BLANK,
    MAGENTA,
};

int main(void)
{
    // Initialize raylib
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Randice - Virtual Dice thrower");
    SetTargetFPS(FPS);
    DisableCursor();

    // Setup objects
    Camera3D camera;
    camera.position = { 10.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 70.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Object ground(
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f },
        { 100.0f, 1.0f, 100.0f },
        PhysicsType::STATIC,
        0.0f,
        GRAY
    );

    std::vector<std::shared_ptr<Object>> cubes;
    Vector3 cubeSize = { 2.0f, 2.0f, 2.0f };

    Physics physics(GRAVITY);
    physics.AddObject(ground);

    for (int i = 0; i < CUBES; ++i) {
        // Vector3 randomPos = {
        //     (float)GetRandomValue(-30, 30),
        //     (float)GetRandomValue(3, 50),
        //     (float)GetRandomValue(-30, 30)
        // };

        Vector3 randomPos = { 0.0f, 10.0f, 0.0f };

        Vector3 randomRot = {
            (float)GetRandomValue(0, 360) * DEG2RAD,
            (float)GetRandomValue(0, 360) * DEG2RAD,
            (float)GetRandomValue(0, 360) * DEG2RAD
        };

        Color randomColor = COLORS[GetRandomValue(0, 24)];

        std::shared_ptr<Object> cube = std::make_shared<Object>(
            randomPos,
            randomRot,
            cubeSize,
            PhysicsType::DYNAMIC,
            1.0f,
            randomColor
        );

        cubes.push_back(cube);

        physics.AddObject(*cube);
    }

    float dt = 1.0f;

    // Main loop
    while (!WindowShouldClose()) {
        // Physics
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        physics.Step(1.0f / FPS * dt * 100.0f);

        // Rendering
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                DrawGrid(1000, 10);
                ground.Render();
                for (auto cube : cubes) {
                    cube->Render();
                }
            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();

        dt = GetFrameTime();
    }

    // Unload objects
    for (int i = 0; i < CUBES; ++i) {
        physics.RemoveObject(*cubes[i]);
        cubes[i]->Unload();
    }

    // Close raylib window
    CloseWindow();

    return 0;
}
