

#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cfloat>

#include "FluidSimulation.h"
#include "TriangleMesh.h"
#include "Timer.h"

int main()
{
    FluidSimulation fluidsim;

    int isize = 64;
    int jsize = 64;
    int ksize = 64;
    float dx = 1.0f / fmax(fmax(isize, jsize), ksize);
    fluidsim.initialize(isize, jsize, ksize, dx);

    TriangleMesh boundaryMesh;
    std::string boundaryMeshPath = "meshes/sphere_large.ply";
    bool invertMesh = true;
    if (!boundaryMesh.loadPLY(boundaryMeshPath))
    {
        std::cout << "Error loading boundary mesh -> " << boundaryMeshPath << std::endl;
        return 0;
    }

    //boundaryMesh.loadASCIIPLY("meshes/cubeBigASCII.ply");

    fluidsim.addBoundary(boundaryMesh, invertMesh);
    TriangleMesh liquidMesh;
    // std::string liquidMeshPath = "meshes/stanford_bunny.ply";
    // if (!liquidMesh.loadPLY(liquidMeshPath))
    // {
    //     std::cout << "Error loading liquid mesh -> " << liquidMeshPath << std::endl;
    //     return 0;
    // }

    liquidMesh.loadASCIIPLY("meshes/armadillo_bigASCII.ply");

    fluidsim.addLiquid(liquidMesh);

    double brickWidth = 2 * dx;
    double brickHeight = brickWidth;
    double brickDepth = brickWidth;

    fluidsim.setGravity(0.0f, -9.81f, 0.0f);
    fluidsim.enableBrickOutput(brickWidth, brickHeight, brickDepth);
    fluidsim.setNumberOfParticlesSeededPerCell(8);
    fluidsim.setFLIPPercentage(0.95f);
    fluidsim.setIsUsingGPU(false);
    int numFrames = 200;
    float timestep = 0.01f;

    Timer total_timer = Timer("Total timer");
    Timer frame_timer = Timer("Frame timer");

    // Create and open a text file
    std::ofstream fluidSimLog("info_logs/fluidsim_log.txt");

    fluidSimLog << "Fluid simulation run: " << std::endl
                << std::endl;

    fluidSimLog << "Brick width height depth: " << brickWidth << std::endl;
    fluidSimLog << "NumParticlesPerCell:" << fluidsim.getNumberOfParticlesSeededPerCell() << std::endl;
    fluidSimLog << "FLIP percentage: " << fluidsim.getFLIPPercentage() << std::endl;
    fluidSimLog << "number of frames" << numFrames << std::endl;
    fluidSimLog << "Using GPU: " << fluidsim.getIsUsingGPU() << std::endl;
    fluidSimLog << "Wyvill kernel used:" << std::boolalpha << (!fluidsim.getIsUsingGPU() && fluidsim._wyvillkernelUsedIfNotUsingGPU) << std::endl;

    total_timer.start();

    fluidSimLog << "Frame times in ms: " << std::endl;

    double total_frame_time = 0;

    for (int frame_num = 0; frame_num <= numFrames; frame_num++)
    {
        std::cout << "Frame " << frame_num << " running " << std::endl;
        frame_timer.restart();
        fluidsim.outputBrickMesh(frame_num, timestep);
        fluidsim.step(timestep);

        std::cout << "Time taken for frame to complete: " << frame_timer.elapsedMilliseconds() << " milliseconds taken." << std::endl;
        std::cout << std::endl;

        total_frame_time += frame_timer.elapsedMilliseconds();

        fluidSimLog
            << frame_timer.elapsedMilliseconds() << std::endl;
    }

    double total_time = total_timer.elapsedMilliseconds();
    frame_timer.stop();
    total_timer.stop();

    std::cout << "Total time taken to compute frames: " << total_time << " milliseconds. " << std::endl;

    fluidSimLog << std::endl
                << "Total time taken to compute frames by using total time timer in ms: " << std::endl;
    fluidSimLog << total_time << std::endl
                << std::endl;

    fluidSimLog << "avg time per frame by using total timer / num frames in ms:" << std::endl;
    fluidSimLog << (total_time / numFrames) << std::endl
                << std::endl;

    fluidSimLog << "Total time by adding up frame times in ms:" << std::endl;
    fluidSimLog << (total_frame_time) << std::endl
                << std::endl;

    fluidSimLog << "avg time per frame by adding up frame times in ms:" << std::endl;
    fluidSimLog << (total_frame_time / numFrames) << std::endl;

    fluidSimLog.close();

    return 0;
}
