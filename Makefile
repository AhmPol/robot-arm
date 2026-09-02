robot_arm:
	gcc main.c helpers.c helpers.h -o robot_arm.exe -I C:/raylib/raylib/include -L C:/raylib/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm

run: robot_arm
	./robot_arm.exe