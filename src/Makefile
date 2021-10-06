# compile and generate executable main file. 
all:
	g++ -std=c++11 -Wall -Wextra main.cpp -o main.exe -std=c++17 TriangleMesh.cpp FluidSimulation.cpp VectorMath.cpp AABB.cpp AABBUtilityFunctions.cpp MeshLevelSet.cpp ParticleLevelSet.cpp MACVelocityField.cpp PressureSolver.cpp Interpolation.cpp  LevelSetFunctions.cpp WaterBrickGrid.cpp GeneralUtilityFunctions.cpp GridIndexVector.cpp
	./main.exe

CUDA: 
	nvcc -std=c++17 -Xcompiler "/std:c++17" main.cpp -o main.exe TriangleMesh.cpp FluidSimulation.cpp VectorMath.cu AABB.cpp AABBUtilityFunctions.cpp MeshLevelSet.cpp ParticleLevelSet.cpp MACVelocityField.cpp PressureSolver.cpp Interpolation.cpp  LevelSetFunctions.cpp WaterBrickGrid.cpp GeneralUtilityFunctions.cpp GridIndexVector.cpp Timer.cpp CUDAFunctions.cu -rdc=true 
	./main

NO_CUDA:
	nvcc -std=c++17 -Xcompiler "/std:c++17" main.cpp -o main.exe TriangleMesh.cpp FluidSimulation.cpp VectorMath.cu AABB.cpp AABBUtilityFunctions.cpp MeshLevelSet.cpp ParticleLevelSet.cpp MACVelocityField.cpp PressureSolver.cpp Interpolation.cpp  LevelSetFunctions.cpp WaterBrickGrid.cpp GeneralUtilityFunctions.cpp GridIndexVector.cpp
	./main


# removes any .exe files from the workspace folder using rm -rf command which is apparently for Mac but seems to work for Windows too. 
clean: 
	rm -rf  *.exe 
	rm -rf brick_colors/*.data
	rm -rf brick_locations/*.obj
	rm -rf blender/blender_progress/*.png
	rm -rf *.exp
	rm -rf *.lib
	rm -rf *.o
	rm -rf info_logs/*.txt



