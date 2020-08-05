#pragma once
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/Control.h>
#include <ompl/control/DirectedControlSampler.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class PathVisibilityChecker
    {

    public:

      PathVisibilityChecker(const ob::SpaceInformationPtr &si);
      ~PathVisibilityChecker(void);

      bool IsPathVisible(std::vector<BundleSpaceGraph::Vertex> &v1, std::vector<BundleSpaceGraph::Vertex> &v2, BundleSpaceGraph::Graph &graph);
      bool IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool IsPathDynamicallyVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2, std::vector<ob::State*> &sLocal);
      bool IsPathVisibleSO2(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool isPathClockwise(std::vector<ob::State*> &spath);
      bool CheckValidity(const std::vector<ob::State*> &s);

      bool IsPathVisibleSimple(std::vector<BundleSpaceGraph::Vertex> &v1, std::vector<BundleSpaceGraph::Vertex> &v2, BundleSpaceGraph::Graph &graph);
      bool IsPathVisibleSimple(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);




      void createStateAt(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const;
      void computePathLength(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength);
      bool isPathDynamicallyFeasible(const std::vector<ompl::base::State*> path) const;

      void testCheckMotion(const ompl::base::State* s1, const ompl::base::State* s2);

      void Test1();
      void Test2();
      void Test3(int F=0);

    protected:
      bool isDynamic{false};
      
      ob::SpaceInformationPtr si_;
      ob::SpaceInformationPtr si_local;
      ob::State *lastValidState;

      ob::StateSpacePtr R2space_;
      ob::RealVectorStateSpace *R2;
      SimpleSetupPtr ss;

      ob::ScopedStatePtr start;
      ob::ScopedStatePtr goal;

      float max_planning_time_path_path{0.2};
      float epsilon_goalregion{0.05};
      
      bool stepFeasible;
      std::vector<ob::State*> statesDyn;
      std::vector<ob::State*> statesDyn_next;

      int controlSamples{20};
      double pathSamples{10.};
      ompl::control::DirectedControlSamplerPtr sDCSampler;
      ompl::control::SpaceInformation* siC;

    private:
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy);
      std::vector<ob::State*> StatesFromVectorSO2R1( 
          const std::vector<double> &st, 
          const std::vector<double> &sx);
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy, 
          const std::vector<double> &st);

    };

  }
}
