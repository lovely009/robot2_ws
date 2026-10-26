#include <gtest/gtest.h>

#include <pinocchio/multibody/model.hpp>

TEST(PinocchioDependencyTest, ConstructsEmptyModel) {
  pinocchio::Model model;

  EXPECT_EQ(model.nq, 0);
  EXPECT_EQ(model.nv, 0);
}
