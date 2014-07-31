#include <iostream>
#include <vector>
#include "vertigo/g2o/edge_se2Switchable.h"
#include "vertigo/g2o/edge_se2MaxMixture.h"
#include "vertigo/g2o/edge_switchPrior.h"
#include "vertigo/g2o/vertex_switchLinear.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam2d/types_slam2d.h>

#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(vertigo);

// ROS stuff
ros::Publisher marker_publisher_;

// G2O optimization
g2o::SparseOptimizer* optimizer_;

// Some handler structs for parsing the datafile in a way that sets us up to process incrementally
struct Pose
{
  int id;
  double x,y,th;
};

struct Edge
{
  int i,j;         // nodes
  double x,y,th;   // measurement
  bool switchable; 
  bool maxMix; 
  Eigen::MatrixXd informationMatrix;
  double weight;
  int switchId;
};

/* Publish the graph */
void publishGraphVisualization()
{
  visualization_msgs::MarkerArray marray;

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = "map";
  loop_edge.header.stamp = ros::Time::now();
  loop_edge.action = visualization_msgs::Marker::ADD;
  loop_edge.ns = "karto";
  loop_edge.id = 0;
  loop_edge.type = visualization_msgs::Marker::LINE_STRIP;
  loop_edge.scale.x = 0.1;
  loop_edge.scale.y = 0.1;
  loop_edge.scale.z = 0.1;
  loop_edge.color.a = 1.0;
  loop_edge.color.r = 1.0;
  loop_edge.color.g = 0.0;
  loop_edge.color.b = 1.0;

  int loop_id = 0;
  int id = 0;
  m.action = visualization_msgs::Marker::ADD;

  std::set<int> vertex_ids;

  for(g2o::SparseOptimizer::EdgeSet::iterator edge_it = optimizer_->edges().begin(); edge_it != optimizer_->edges().end(); ++edge_it)
  {
    EdgeSE2Switchable* edge_switch = dynamic_cast<EdgeSE2Switchable*>(*edge_it);

    int id1, id2;
    g2o::VertexSE2* v1, *v2; 

    bool is_loop_edge; 
    double alpha;

    if(edge_switch != NULL)
    {
      // Switchable loop closure
      v1 = dynamic_cast<g2o::VertexSE2 *>(edge_switch->vertices()[0]);
      v2 = dynamic_cast<g2o::VertexSE2 *>(edge_switch->vertices()[1]);
      VertexSwitchLinear *v3 = dynamic_cast<VertexSwitchLinear *>(edge_switch->vertices()[2]);
      loop_edge.color.a = 1.0;
      if(v3->x() < 0.5) // switched off
      {
        loop_edge.color.a = 0.25;
      }
      is_loop_edge = true;
    }
    else  // MAXMIX?
    {
      EdgeSE2MaxMixture* edge_maxmix = dynamic_cast<EdgeSE2MaxMixture*>(*edge_it);
      
      if(edge_maxmix != NULL)
      {
        v1 = dynamic_cast<g2o::VertexSE2 *>(edge_maxmix->vertices()[0]);
        v2 = dynamic_cast<g2o::VertexSE2 *>(edge_maxmix->vertices()[1]);
        loop_edge.color.a = 1.0;
        if(edge_maxmix->nullHypothesisMoreLikely)
          loop_edge.color.a = 0.25;
        is_loop_edge = true;
      }
      else // ODOMETRY?
      {
        g2o::EdgeSE2* edge_odo = dynamic_cast<g2o::EdgeSE2*>(*edge_it);
        if(edge_odo != NULL)
        {
          v1 = dynamic_cast<g2o::VertexSE2 *>(edge_odo->vertices()[0]);
          v2 = dynamic_cast<g2o::VertexSE2 *>(edge_odo->vertices()[1]);
          is_loop_edge = false;
        }
      }
    }
    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

    if(is_loop_edge)
    {
      loop_edge.points.clear();
      loop_edge.points.push_back(p1);
      loop_edge.points.push_back(p2);
      loop_edge.id = id;
      marray.markers.push_back(visualization_msgs::Marker(loop_edge));
    }
    else
    { 
      edge.points.clear();
      edge.points.push_back(p1);
      edge.points.push_back(p2);
      edge.id = id;
      marray.markers.push_back(visualization_msgs::Marker(edge));
    }
    id++;

    // Check the vertices exist, if not add
    if( vertex_ids.find(v1->id()) == vertex_ids.end() )
    {
      // Add the vertex to the marker array 
      m.id = id;
      m.pose.position.x = p1.x;
      m.pose.position.y = p1.y;
      vertex_ids.insert(v1->id());
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    } 
    // Check the vertices exist, if not add
    if( vertex_ids.find(v2->id()) == vertex_ids.end() )
    {
      // Add the vertex to the marker array 
      m.id = id;
      m.pose.position.x = p2.x;
      m.pose.position.y = p2.y;
      vertex_ids.insert(id2);
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    }
  }
  marker_publisher_.publish(marray);
}

void visualizerThread()
{
  ros::Rate r(0.5);
  while(ros::ok())
  {
    publishGraphVisualization();
    r.sleep();
  }
}

/* This function creates an "ordered" list of poses and edges that can be used for incremental construction
 * of the map
 */
bool parseDataFile(const char *filename, vector<Pose> &poses, vector<Edge> &edges, multimap<int, int> &poseToEdges)
{
  ifstream inFile(filename);
  if(!inFile)
  {
    ROS_ERROR("Can't open file: %s", filename);
    return false;
  }

  while(!inFile.eof())
  {
    // Handle things differently depending on the type of vertex or edge
    string type;
    inFile >> type;

    if(type == "VERTEX_SE2")  // Odometry edge
    {
      Pose p;
      inFile >> p.id >> p.x >> p.y >> p.th;
      poses.push_back(p);
    }
    else if(type == "EDGE_SE2_SWITCHABLE" || type == "EDGE_SE2" || type == "EDGE_SE2_MAXMIX")
    {
      Edge e;
      // read pose IDs
      inFile >> e.i >> e.j;

      if(e.i > e.j)  // reorder
      {
        swap(e.i,e.j);
      }

      // read the switch variable ID
      if (type == "EDGE_SE2_SWITCHABLE") inFile >> e.switchId;
      if (type == "EDGE_SE2_MAXMIX") inFile >> e.weight;

      // read odometry measurement
      inFile >> e.x >> e.y >> e.th;

      // read information matrix
      double info[6];
      inFile >> info[0] >> info[1] >> info[2] >> info[3] >> info[4] >> info[5];
      Eigen::MatrixXd informationMatrix(3,3);
      informationMatrix << info[0], info[1], info[2], info[1], info[3], info[4], info[2], info[4], info[5];
      e.informationMatrix = informationMatrix;

      e.switchable = false;
      e.maxMix= false;

      if(type == "EDGE_SE2_SWITCHABLE") e.switchable = true;
      else if(type == "EDGE_SE2_MAXMIX") e.maxMix = true;

      edges.push_back(e);

      int id=edges.size()-1;
      poseToEdges.insert(pair<int,int>(e.j, id));  // Build up a map of incident edges for each pose
    }

    // If it's another class (VERTEX_SWITCH/EDGE_SWITCH_PRIOR ... just ignore
  }
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vertigo_incremental_test");

  ros::NodeHandle node_;
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  if(argc < 2)
  {
    ROS_ERROR("Usage: rosrun vertigo incremental_optimization data_filename");
    return 0;
  }
  std::string inputFile(argv[1]);

  // Set up G2O
  optimizer_ = new g2o::SparseOptimizer();
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>             SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  optimizer_->setAlgorithm(solverGauss);

  // Read in datafile
  std::vector<Pose> poses;
  std::vector<Edge> edges;
  multimap<int,int> poseToEdges;
  if(!parseDataFile(inputFile.c_str(), poses, edges, poseToEdges))
    return -1;


  ROS_INFO("Loaded %d poses and %d edges", (int)poses.size(), (int)edges.size());
  ROS_INFO("Running incremental G2O ...");

  // Spin up a visualization thread
  boost::thread visualizer_thread(&visualizerThread);

  // Iterate through the poses and incrementally build and solve the graph
  for(std::vector<Pose>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
    Pose p = *it;
    // Add the pose to the graph
    g2o::VertexSE2* vertex = new g2o::VertexSE2();
    vertex->setId(p.id);
    g2o::SE2 pose(p.x,p.y,p.th);
    vertex->setEstimate(pose);

    if(optimizer_->vertices().size() == 0)
      vertex->setFixed(true);

    optimizer_->addVertex(vertex);

    // Get all the edges that involve this pose      
    pair<multimap<int, int>::iterator, multimap<int, int>::iterator > ret = poseToEdges.equal_range(p.id);

    for(multimap<int, int>::iterator it=ret.first; it!=ret.second; it++)
    {
      Edge e = edges[it->second];

      // Is this an odometry edge?
      if(e.j==e.i+1)
      {
        g2o::EdgeSE2* edge = new g2o::EdgeSE2();
        edge->vertices()[0] = optimizer_->vertex(e.i);
        edge->vertices()[1] = optimizer_->vertex(e.j);
        g2o::SE2 pose(e.x,e.y,e.th);
        edge->setMeasurement(pose);
        edge->setInformation(e.informationMatrix);
        optimizer_->addEdge(edge);
      }
      else
      {
        if(e.switchable)
        {
          // Create a Vertex
          VertexSwitchLinear* vertex = new VertexSwitchLinear();
          vertex->setId(e.switchId);
          double init_value = 1.0;
          vertex->setEstimate(init_value);
          optimizer_->addVertex(vertex);

          // Create a Unary Edge
          EdgeSwitchPrior *edge_prior = new EdgeSwitchPrior();
          edge_prior->vertices()[0] = optimizer_->vertex(e.switchId);
          edge_prior->setMeasurement(1.0);
          edge_prior->setInformation(Eigen::MatrixXd::Identity(1,1));
          optimizer_->addEdge(edge_prior);

          // Create a switchable edge
          EdgeSE2Switchable *edge_loop = new EdgeSE2Switchable();
          edge_loop->vertices()[0] = optimizer_->vertex(e.i);
          edge_loop->vertices()[1] = optimizer_->vertex(e.j);
          edge_loop->vertices()[2] = optimizer_->vertex(e.switchId);
          g2o::SE2 pose(e.x,e.y,e.th);
          edge_loop->setMeasurement(pose);
          edge_loop->setInformation(e.informationMatrix);
          optimizer_->addEdge(edge_loop);

          optimizer_->initializeOptimization();
          optimizer_->optimize(30);
        }
        else if(e.maxMix)
        {
          // Create a MaxMixture edge
          EdgeSE2MaxMixture *edge_maxmix = new EdgeSE2MaxMixture();
          edge_maxmix->vertices()[0] = optimizer_->vertex(e.i);
          edge_maxmix->vertices()[1] = optimizer_->vertex(e.j);

          g2o::SE2 pose(e.x,e.y,e.th);
          edge_maxmix->setMeasurement(pose);
          edge_maxmix->setInformation(e.informationMatrix);
          edge_maxmix->weight = 1e-7;
          edge_maxmix->information_constraint = e.informationMatrix;
          edge_maxmix->nu_constraint = 1.0/sqrt(e.informationMatrix.inverse().determinant());
          edge_maxmix->information_nullHypothesis = e.informationMatrix*edge_maxmix->weight;
          edge_maxmix->nu_nullHypothesis = 1.0/sqrt(e.informationMatrix.inverse().determinant());

          optimizer_->addEdge(edge_maxmix);

          optimizer_->initializeOptimization();
          optimizer_->optimize(30);
        }
      }
    } // Edges
    if( (p.id % 100) == 0)
      ROS_INFO("Processed %d nodes",p.id);
  } // Vertices

  ROS_INFO("Optimization done");
  ros::shutdown();
  visualizer_thread.join();
  return 0;
}

