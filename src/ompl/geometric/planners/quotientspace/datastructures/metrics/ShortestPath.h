#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_SHORTEST_PATH_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_SHORTEST_PATH_
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/Geodesic.h>

namespace ompl
{
  namespace geometric
  {
    OMPL_CLASS_FORWARD(PathGeometric);
    class BundleSpaceMetricShortestPath: public BundleSpaceMetricGeodesic
    {

        using BaseT = BundleSpaceMetricGeodesic;

      public:

        BundleSpaceMetricShortestPath() = delete;
        BundleSpaceMetricShortestPath(BundleSpaceGraph*); 
        virtual ~BundleSpaceMetricShortestPath() override;

        virtual double distanceBundle(
            const Configuration *xStart, 
            const Configuration *xDest) override;
        virtual double distanceFiber(
            const Configuration *xStart, 
            const Configuration *xDest) override;
        virtual double distanceBase(
            const Configuration *xStart, 
            const Configuration *xDest) override;

        std::vector<const Configuration*> getInterpolationPath(
            const Configuration *xStart, 
            const Configuration *xDest);

        virtual void interpolateBundle(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp) override;

      protected:
        std::vector<Configuration*> tmpPath_;
        Configuration *xBaseStart_{nullptr};
        Configuration *xBaseDest_{nullptr};// = new Configuration(base, xBaseDestTmp_);
    };
  }
}

#endif
