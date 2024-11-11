#include <iostream>
#include <chrono>


// Example function to measure
void exampleFunction() {
    // Simulate some work
    for (volatile int i = 0; i < 1e6; ++i); // Volatile to prevent optimization
}

int main() {
    // Start measuring time
    auto start = std::chrono::high_resolution_clock::now();

    // Call the function
    exampleFunction();

    // Stop measuring time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    std::chrono::duration<double, std::milli> duration = end - start;

    // Output the result
    std::cout << "Function execution time: " << duration.count() << " ms" << std::endl;

    return 0;
}
