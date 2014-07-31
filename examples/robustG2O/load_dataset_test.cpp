#include <iostream>
#include <vector>
#include "vertigo/g2o/edge_se2Switchable.h"
#include "vertigo/g2o/edge_switchPrior.h"
#include "vertigo/g2o/vertex_switchLinear.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam2d/types_slam2d.h>

#include <ros/ros.h>

using namespace std;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(vertigo);

g2o::SparseOptimizer* optimizer_;
unsigned int marker_count_ = 0;
ros::Publisher marker_publisher_;

bool loadFromFile(const char *filename)
{
  ifstream inFile(filename);
  if(!inFile.good())
  {
    ROS_ERROR("Can't open file: %s", filename);
    return false;
  }

  ROS_INFO("Loading ...");
  optimizer_->load(filename);
  ROS_INFO("Loaded graph with %d vertices and %d edges", (int)optimizer_->vertices().size(),(int)optimizer_->edges().size()); 
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vertigo_load_dataset_test");
  ros::NodeHandle node_;

  std::string inputFile(argv[1]);
 
  // Set up G2O solver
  optimizer_ = new g2o::SparseOptimizer();
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  optimizer_->setAlgorithm(solverGauss);

  if(!loadFromFile(inputFile.c_str()))
    return -1;

  ROS_INFO("Optimizing ...");
  optimizer_->initializeOptimization();
  optimizer_->optimize(30);
  ROS_INFO("Done!");
 return 0;
}

