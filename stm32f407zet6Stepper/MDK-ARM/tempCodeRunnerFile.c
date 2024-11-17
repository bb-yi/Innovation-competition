#include <assert.h>
#include <math.h>

void test_smooth_speed() {
    // Test case 1: Happy path
    float result = smooth_speed(0.5, 100.0, 50.0, 10.0);
    assert(fabs(result - 50.0) < 0.001); // Expect target speed to be 50.0

    // Test case 2: Alpha = 0
    result = smooth_speed(0.0, 100.0, 60.0, 10.0);
    assert(fabs(result - 10.0) < 0.001); // Expect target speed to be 10.0

    // Test case 3: Alpha = 1
    result = smooth_speed(1.0, 100.0, 60.0, 10.0);
    assert(fabs(result - 60.0) < 0.001); // Expect target speed to be 60.0

    // Test case 4: Low run distance
    result = smooth_speed(0.5, 0.1, 50.0, 10.0);
    assert(fabs(result - 1.0) < 0.001); // Expect target speed to be 1.0 (clamped)

    // Test case 5: High acceleration
    result = smooth_speed(0.5, 100.0, 200.0, 100.0);
    assert(fabs(result - 100.0) < 0.001); // Expect target speed to be 100.0 

    // Test case 6: Run distance negative
    result = smooth_speed(0.5, -100.0, 50.0, 10.0);
    assert(fabs(result - 1.0) < 0.001); // Expect target speed to be 1.0 (clamped)

    // Test case 7: Speed below minimum
    result = smooth_speed(0.5, 100.0, 0.5, 10.0);
    assert(fabs(result - 1.0) < 0.001); // Expect target speed to be 1.0 (clamped)
}

int main() {
    test_smooth_speed();
    return 0;
}