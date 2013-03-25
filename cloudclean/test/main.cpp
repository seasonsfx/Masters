#include "gtest/gtest.h"
#include "utilities/pointpicker.h"

namespace {

// The fixture for testing class Foo.
class FooTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.
  Eigen::Vector4f world_point;
  Eigen::Vector4f projected_point;
  Eigen::Affine3f proj;
  Eigen::Affine3f mv;
  int win_width;
  int win_height;

  FooTest() {
      world_point = Eigen::Vector4f(0.1, 0.3, 0.7, 1);
    // You can do set-up work for each test here.
  }

  virtual ~FooTest() {
    EXPECT_EQ(1, 1);
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(FooTest, MethodBarDoesAbc) {
    EXPECT_EQ(1, 1);
}

// Tests that Foo does Xyz.
TEST_F(FooTest, DoesXyz) {
  EXPECT_EQ(0, 0);
  // Exercises the Xyz feature of Foo.
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
