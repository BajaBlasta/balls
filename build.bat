@echo off
gcc main.c -I../tools/raylib/src -L../tools/raylib/build/raylib -lraylib -lopengl32 -lgdi32 -lwinmm -std=c11 -O2 -o balls.exe
