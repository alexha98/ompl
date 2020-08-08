#include <ompl/geometric/planners/multilevel/algorithms/ExplorerTestImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include "../ExplorerTestImpl.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/explorer/PathVisibilityChecker.h>
#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <chrono>

#define foreach BOOST_FOREACH

ompl::geometric::ExplorerTestImpl::ExplorerTestImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("ExplorerTestImpl" + std::to_string(id_));
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());
}

ompl::geometric::ExplorerTestImpl::~ExplorerTestImpl()
{
    delete pathVisibilityChecker_;
}

void ompl::geometric::ExplorerTestImpl::grow()
{
    auto start = std::chrono::system_clock::now();

    if (firstRun_)
    {
        auto start = std::chrono::system_clock::now();
        pathlist.open("paths.csv");
        pathlist << "Iteration"
                 << ",";
        pathlist << "Zeit"
                 << ",";
        pathlist << "Vertices"
                 << ",";
        pathlist << "Edges"
                 << ",";
        pathlist << "#Path" << ",";
        pathlist << "Paths"
                 << "\n";
        ExplorerTestImpl::firstGrow();
    }
    else
    {
        iteration++;
        std::cout << "ITERATION " << iteration << std::endl;
        //(1) Get Random Sample
        if (!sampleBundleValid(xRandom_->state))
            return;

        //(2) Add Configuration if valid
        Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
        addConfiguration(xNew);

        //(3) Connect to K nearest neighbors
        connectNeighbors(xNew);

        // expand();

        if (!hasSolution_)
        {
            if (sameComponent(vStart_, vGoal_))
            {
                hasSolution_ = true;
            }
        }

        Vertex v = xNew->index;

        // FALL 1: Pfad ist mit Start und Ziel verbunden
        bool b = sameComponent(v, vStart_) && sameComponent(v, vGoal_);

        if (b)
        {
            ompl::base::PathPtr pPath = getPath(vStart_, v);
            std::vector<Vertex> path = shortestVertexPath_;
            // og::PathGeometric &gpath = static_cast<og::PathGeometric &>(*pPath);

            ompl::base::PathPtr pPath2 = getPath(v, vGoal_);
            std::vector<Vertex> path2 = shortestVertexPath_;
            // og::PathGeometric &gpath2 = static_cast<og::PathGeometric &>(*pPath2);

            float pathlength = pPath->length() + pPath2->length();
            path.insert(path.end(), path2.begin() + 1, path2.end());

            // std::cout << graph_[path.at(0)]->state << std::endl;

            std::vector<Vertex> tempPath = path;
            sort(tempPath.begin(),tempPath.end());
            auto it = std::unique(tempPath.begin(),tempPath.end());
            bool noduplicates = (it == tempPath.end());

            if (noduplicates)
            {
                bool isVisible = false;

                for (uint i = 0; i < solutionspaths.size(); i++)
                {
                    // FALL 2: verbessert bestehenden pfad
                    // Knoten wurde mit 2 Knoten des Pfades verbunden
                    if (pathVisibilityChecker_->IsPathVisibleSimple(path, solutionspaths.at(i), graph_))
                    {
                        std::cout << "ITS VISIBLE" << std::endl;
                        isVisible = true;
                        // Path is shorter than visible path else do nothing
                        if (pathlength < solutionPathLength.at(i))
                        {
                            solutionspaths.at(i) = path;
                            solutionPathLength.at(i) = pathlength;
                        }
                        break;
                    }
                }

                if (!isVisible)
                {
                    std::cout << "NOT VISIBLE" << std::endl;
                    solutionspaths.push_back(path);
                    solutionPathLength.push_back(pathlength);
                    newpaths.push_back(solutionspaths.size() - 1);

                    std::cout << "########################################" << std::endl;
                    std::cout << "NEWPATH: " << newpaths.size() << std::endl;
                    std::cout << "########################################" << std::endl;

                }
            }
        }
        // FALL 3: neuen Pfad
        // Knoten kann mit Start und Ziel verbunden werden
    }

    for (uint i = 0; i < solutionspaths.size(); i++)
    {
        std::cout << "Path: ";
        for (uint j = 0; j < solutionspaths.at(i).size(); j++)
        {
            std::cout << solutionspaths.at(i).at(j) << ' ';
        }
        std::cout << "\t Length: ";
        std::cout << solutionPathLength.at(i) << std::endl;
    }

    if (solutionspaths.size() > 0)
    {
        pathStackHead_.clear();
        for (uint i = 0; i < solutionspaths.size(); i++)
        {
            std::vector<ompl::base::State *> temp;
            for (uint j = 0; j < solutionspaths.at(i).size(); ++j)
            {
                temp.push_back(graph_[solutionspaths.at(i).at(j)]->state);
            }
            pathStackHead_.push_back(temp);
        }
    }

    if (((iteration % 100) == 0) && !(newpaths.empty()))
    {
        std::cout << "#################################################################################################"
                     "#######"
                  << std::endl;
        std::cout << "########################################" << std::endl;
        std::cout << "NEWPATH: " << newpaths.size() << std::endl;
        std::cout << "########################################" << std::endl;
        for (int i = 0; i < newpaths.size(); ++i)
        {
            for (int j = 0; j < solutionspaths.size(); ++j)
            {
                if (newpaths.at(i) != j)
                {
                    if (pathVisibilityChecker_->IsPathVisible(solutionspaths.at(newpaths.at(i)), solutionspaths.at(j),
                                                              graph_))
                    {
                        std::cout<<"VISIBLE"<<std::endl;
                        std::cout << "i: " << newpaths.at(i) << " j: " << j << std::endl;

                        if (solutionPathLength.at(i) <= solutionPathLength.at(j))
                        {
                            std::cout << "SHORTER j" << std::endl;
                                erasepaths.push_back(j);

                        }
                        else
                        {
                            std::cout << "SHORTER i" << std::endl;

                            erasepaths.push_back(newpaths.at(i));
                        }
                        break;
                    }
                }
            }
        }
        std::cout << "########################################" << std::endl;
        std::cout << "ERASE SIZE : " << erasepaths.size() << std::endl;
        std::cout << "########################################" << std::endl;
        sort(erasepaths.begin(),erasepaths.end());

        erasepaths.erase(std::unique(erasepaths.begin(), erasepaths.end()), erasepaths.end());

        std::cout << "########################################" << std::endl;
        std::cout << "ERASE SIZE : " << erasepaths.size() << std::endl;
        std::cout << "########################################" << std::endl;
        reverse(erasepaths.begin(), erasepaths.end());

        for (int i = 0; i < erasepaths.size(); i++)
        {
            std::cout << "ERASE: " << erasepaths.at(i) << std::endl;
            solutionspaths.erase(solutionspaths.begin() + erasepaths.at(i));
            solutionPathLength.erase(solutionPathLength.begin() + erasepaths.at(i));
        }
        erasepaths.clear();
        newpaths.clear();
    }

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    elapsed_all_seconds = elapsed_all_seconds + elapsed_seconds;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    std::cout << "ALL time: " << elapsed_all_seconds.count() << "s\n";

    pathlist << iteration << ",";
    pathlist << elapsed_all_seconds.count() << ",";
    pathlist << getNumberOfVertices() << ",";
    pathlist << getNumberOfEdges() << ",";
    pathlist << solutionspaths.size() << ",";

    for (int i = 0; i < solutionPathLength.size(); ++i)
    {
        pathlist << solutionPathLength.at(i) << ",";
    }
    pathlist << "\n";

    if (iteration > 1000)
    {
        pathlist.close();
    }
}

void ompl::geometric::ExplorerTestImpl::firstGrow()
{
    std::cout << "FIRST" << std::endl;

    init();
    vGoal_ = addConfiguration(qGoal_);
    firstRun_ = false;

    if (hasBaseSpace())
    {
        if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
        {
            if (sameComponent(vStart_, vGoal_))
            {
                hasSolution_ = true;
            }
        }
    }

    while (!hasSolution_)
    {
        iteration++;
        //(1) Get Random Sample
        if (!sampleBundleValid(xRandom_->state))
            return;

        //(2) Add Configuration if valid
        Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
        addConfiguration(xNew);

        //(3) Connect to K nearest neighbors
        connectNeighbors(xNew);

        // expand();

        if (!hasSolution_)
        {
            if (sameComponent(vStart_, vGoal_))
            {
                hasSolution_ = true;

                Vertex v = xNew->index;

                ompl::base::PathPtr pPath = getPath(vStart_, v);
                std::vector<Vertex> path = shortestVertexPath_;

                ompl::base::PathPtr pPath2 = getPath(v, vGoal_);
                std::vector<Vertex> path2 = shortestVertexPath_;

                path.insert(path.end(), path2.begin() + 1, path2.end());
                float pathlength = pPath->length() + pPath2->length();

                solutionspaths.push_back(path);
                solutionPathLength.push_back(pathlength);

                pathlist << iteration << ",";

                for (int i = 0; i < solutionPathLength.size(); ++i)
                {
                    pathlist << solutionPathLength.at(i) << ",";
                }
                pathlist << "\n";
            }
        }
    }
}

void ompl::geometric::ExplorerTestImpl::getPlannerData(ompl::base::PlannerData &data) const
{
    if (isDynamic())
    {
        if (!data.hasControls())
        {
            OMPL_ERROR("Dynamic Cspace, but PlannerData has no controls.");
        }
    }
    if (pathStackHead_.size() > 0)
    {
        OMPL_DEVMSG1("%s has %d solutions.", getName().c_str(), pathStackHead_.size());
        if (pathStackHead_.empty())
        {
            OMPL_ERROR("%s has 0 solutions.", getName().c_str());
            throw ompl::Exception("Zero solutions");
        }
        std::vector<int> idxPathI;
        for (uint i = 0; i < pathStackHead_.size(); i++)
        {
            const std::vector<ob::State *> states = pathStackHead_.at(i);

            idxPathI.clear();
            idxPathI.push_back(i);

            //############################################################################
            // DEBUG
            /*std::cout << "[";
            for (uint k = 0; k < idxPathI.size(); k++)
            {
                std::cout << idxPathI.at(k) << " ";
            }
            std::cout << "]" << std::endl;*/
            //############################################################################

            ob::PlannerDataVertexAnnotated *p1 =
                new ob::PlannerDataVertexAnnotated(getBundle()->cloneState(states.at(0)));
            p1->setLevel(level_);
            p1->setPath(idxPathI);
            data.addStartVertex(*p1);

            for (uint k = 0; k < states.size() - 1; k++)
            {
                getBundle()->printState(states.at(k));

                ob::PlannerDataVertexAnnotated *p2 =
                    new ob::PlannerDataVertexAnnotated(getBundle()->cloneState(states.at(k + 1)));
                p2->setLevel(level_);
                p2->setPath(idxPathI);

                if (k == states.size() - 2)
                {
                    data.addGoalVertex(*p2);
                }
                else
                {
                    data.addVertex(*p2);
                }
                data.addEdge(*p1, *p2);

                p1 = p2;
            }
        }

        // GET PLANNER ROADMAP DATA
        foreach (const Edge e, boost::edges(graph_))
        {
            const Vertex v1 = boost::source(e, graph_);
            const Vertex v2 = boost::target(e, graph_);

            base::PlannerDataVertexAnnotated p1(graph_[v1]->state);
            base::PlannerDataVertexAnnotated p2(graph_[v2]->state);
            p1.setPath(idxPathI);
            p2.setPath(idxPathI);
            data.addEdge(p1, p2);
        }
        foreach (const Vertex v, boost::vertices(graph_))
        {
            base::PlannerDataVertexAnnotated p(graph_[v]->state);
            p.setPath(idxPathI);
            data.addVertex(p);
        }
    }
}

int ompl::geometric::ExplorerTestImpl::getSelectedPath()
{
    return selectedPath_;
}

void ompl::geometric::ExplorerTestImpl::setSelectedPath(int selectedPath)
{
    selectedPath_ = selectedPath;
    std::cout << "Level" << level_ << " set new selected path to " << selectedPath_ << std::endl;
}

unsigned int ompl::geometric::ExplorerTestImpl::getNumberOfPaths() const
{
    return pathStackHead_.size();
}
