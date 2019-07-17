#include "KinematicChain.h"
#include "QuotientSpacePlanningCommon.h"
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

const unsigned int numLinks = 7;
const double linkLength = 1.0 / numLinks;
const double narrowPassageWidth = 0.2;

namespace ot = ompl::tools;
std::vector<Environment> envs;

ob::PlannerPtr GetQRRT(
    ob::SpaceInformationPtr si, 
    ob::ProblemDefinitionPtr pdef, 
    std::vector<double> start, 
    std::vector<double> goal, 
    uint numLinks, 
    Environment &env)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;
    std::vector<ob::ProblemDefinitionPtr> pdef_vec;

    for(uint k = 3; k < numLinks; k+=2)
    {
        OMPL_INFORM("Create QuotientSpace Chain with %d links.", k);
        Environment envk = createHornEnvironment(k, narrowPassageWidth);
        auto chainK(std::make_shared<KinematicChainSpace>(k, linkLength, &envk));
        envs.push_back(envk);

        auto siK = std::make_shared<ob::SpaceInformation>(chainK);
        siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
        ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(siK);

        uint clippedDofs = numLinks - k;
        std::vector<double> startVecK(start.begin(), start.end()-clippedDofs);
        std::vector<double> goalVecK(goal.begin(), goal.end()-clippedDofs);

        ompl::base::ScopedState<> startk(chainK), goalk(chainK);
        chainK->setup();
        chainK->copyFromReals(startk.get(), startVecK);
        chainK->copyFromReals(goalk.get(), goalVecK);
        pdefk->setStartAndGoalStates(startk, goalk);

        si_vec.push_back(siK);
        pdef_vec.push_back(pdefk);
    }
    OMPL_INFORM("Add Original Chain with %d links.", numLinks);
    si_vec.push_back(si);
    pdef_vec.push_back(pdef);

    typedef og::MultiQuotient<og::QRRT> MultiQuotient;
    auto planner = std::make_shared<MultiQuotient>(si_vec);
    planner->setProblemDefinition(pdef_vec);
    planner->setName("QuotientSpaceRRT");
    return planner;
}

int main()
{
    Environment env = createHornEnvironment(numLinks, narrowPassageWidth);
    OMPL_INFORM("Original Chain has %d links", numLinks);
    auto chain(std::make_shared<KinematicChainSpace>(numLinks, linkLength, &env));
    ompl::geometric::SimpleSetup ss(chain);

    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));

    ompl::base::ScopedState<> start(chain), goal(chain);
    std::vector<double> startVec(numLinks, boost::math::constants::pi<double>() / (double)numLinks);
    std::vector<double> goalVec(numLinks, 0);

    startVec[0] = 0.;
    goalVec[0] = boost::math::constants::pi<double>() - .001;
    chain->setup();
    chain->copyFromReals(start.get(), startVec);
    chain->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));

    ob::PlannerPtr quotientSpacePlanner = 
      GetQRRT(ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec, 
          numLinks, 
          env);

    b.addPlanner( quotientSpacePlanner );

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    PrintBenchmarkResults(b);
    return 0;
}