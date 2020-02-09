#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE2RN_SE2__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE2RN_SE2__
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_SE2RN_SE2: public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;
          public:
            BundleSpaceComponent_SE2RN_SE2(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const override;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override;

            virtual void mergeStates(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override;

          protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;


        };
    }
}

#endif

