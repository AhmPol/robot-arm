robot_arm:
	gcc main.c helpers.c helpers.h -o robot_arm.exe -I C:/raylib/raylib/src -L C:/raylib/raylib/src -lraylib -lopengl32 -lgdi32 -lwinmm

run: robot_arm
	./robot_arm.exe