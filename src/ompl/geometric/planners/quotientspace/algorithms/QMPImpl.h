/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_GEOMETRIC_PLANNERS_BundleSpace_QMPIMPL_
#define OMPL_GEOMETRIC_PLANNERS_BundleSpace_QMPIMPL_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        /** \brief Implementation of BundleSpace Rapidly-Exploring Random Trees Algorithm*/
        class QMPImpl : public ompl::geometric::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

        public:
            QMPImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~QMPImpl() override;
            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;
            virtual bool getSolution(ompl::base::PathPtr &solution) override;
            /** \brief Importance based on how many vertices the tree has */
            double getImportance() const override;

            /** \brief Uniform sampling */
            virtual bool sampleBundle(ompl::base::State *q_random) override;
            /** \brief Uniform sampling with goal bias*/
            bool sampleBundleGoalBias(Configuration*);

            /** \brief \brief Quotient-Space sampling by choosing a random vertex from parent
                class tree */
            virtual bool sampleFromDatastructure(ompl::base::State *) override;

            virtual void setup() override;
            virtual void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double distance);
            double getRange() const;

        protected:
            /** \brief Random configuration placeholder */
            Configuration *qRandom_{nullptr};
            /** \brief Current shortest path on tree */
            std::vector<Vertex> shortestPathVertices_;

            /** \brief Maximum distance of expanding the tree */
            double maxDistance_{.0};
            /** \brief Goal bias similar to RRT */
            double goalBias_{.05};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
