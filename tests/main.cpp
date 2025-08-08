#include <gtest/gtest.h>

// A simple function to test
auto add(int a, int b) -> int { return a + b; }

// Test case: Check addition works correctly
TEST(MathTests, AddPositiveNumbers) { EXPECT_EQ(add(2, 3), 5); }

TEST(MathTests, AddNegativeNumbers) { EXPECT_EQ(add(-2, -3), -5); }

TEST(MathTests, AddMixedSignNumbers) { EXPECT_EQ(add(-2, 3), 1); }

// Test case: Intentional failure example
TEST(FailureExample, AlwaysFails) {
  EXPECT_EQ(1, 0) << "This test is supposed to fail.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
