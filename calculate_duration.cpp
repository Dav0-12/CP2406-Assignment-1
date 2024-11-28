#include <iostream>
#include <chrono>
#include "nbody/BarnzNhutt.cpp"

int main() {

    // Call the function
    #ifdef FE_NOMASK_ENV
    if (DEBUG_INFO)
        // enable all hardware floating point exceptions for debugging
            feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
    #endif
    std::cout << SYSTEM_THICKNESS << "AU thick disk\n";;
    char *image = new char[WIDTH * HEIGHT * 3];
    double *hdImage = new double[WIDTH * HEIGHT * 3];
    struct body *bodies = new struct body[NUM_BODIES];

    initializeBodies(bodies);

    // Start measuring time
    auto start = std::chrono::high_resolution_clock::now();

    runSimulation(bodies, image, hdImage);

    // Stop measuring time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    std::chrono::duration<double, std::milli> duration = end - start;

    // Output the result
    std::cout << "Function execution time: " << duration.count() << " ms" << std::endl;

    return 0;
}
