#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_DATASTRUCTURES_GRAPHSAMPLER_RANDOM_VERTEX_
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceGraphSamplerRandomVertex : public BundleSpaceGraphSampler
        {
            using BaseT = BundleSpaceGraphSampler;

        public:
            BundleSpaceGraphSamplerRandomVertex() = delete;
            BundleSpaceGraphSamplerRandomVertex(BundleSpaceGraph *);

        protected:
            virtual void sampleImplementation(base::State *xRandom) override;
        };
    }  // namespace geometric
}  // namespace ompl

#endif