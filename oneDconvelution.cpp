#include <iostream>
#include <vector>
#include <chrono>

int main() {
    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<int> signal(1024);
    for (int i = 0; i < 1024; i++) {
        signal[i] = i + 1;
    }

    std::vector<int> kernel = {1, 1, 1};

    std::vector<int> result(signal.size() - kernel.size() + 1);

    for (size_t i = 0; i < result.size(); i++) {
        int sum = 0;
        for (size_t j = 0; j < kernel.size(); j++) {
            sum += signal[i + j] * kernel[j];
        }
        result[i] = sum;
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);

    std::cout << "Elapsed time: "
              << elapsed.count() << " ms\n";

    return 0;
}