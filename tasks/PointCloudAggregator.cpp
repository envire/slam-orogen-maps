/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointCloudAggregator.hpp"

//#include <vizkit3d/Vizkit3DWidget.hpp>
//#include "MLSGridVisualization.hpp"
//#include <vizkit3d/GridVisualization.hpp>


using namespace envire_maps;

PointCloudAggregator::PointCloudAggregator(std::string const& name)
    : PointCloudAggregatorBase(name)
{
}

PointCloudAggregator::PointCloudAggregator(std::string const& name, RTT::ExecutionEngine* engine)
    : PointCloudAggregatorBase(name, engine)
{
}

PointCloudAggregator::~PointCloudAggregator()
{
}

void PointCloudAggregator::bodyPoseCallback(const base::Time &ts, const ::base::samples::RigidBodyState &bodyPose_sample)
{
    //throw std::runtime_error("Aggregator callback for bodyPose not implemented");
    lastPose = bodyPose_sample;
    std::cout << "B" << std::flush;
}

void PointCloudAggregator::pointCloudCallback(const base::Time &ts, const ::base::samples::Pointcloud &pc_sample)
{
//    throw std::runtime_error("Aggregator callback for pointCloud not implemented");
    const std::vector<base::Point> &points = pc_sample.points;
    size_t N = points.size();
    std::cout << "C " << N << " at " << lastPose.getPose() << std::endl;

    base::Pose body2World = lastPose.getPose();
    Eigen::Affine3d scanner2World = body2World.toTransform() * _scanner_in_body.rvalue().getPose().toTransform();
    base::samples::Pointcloud out;
    envire::maps::PointCloud pc;
    pc.resize(N);
    out.points.reserve(N);
    for(size_t i=0; i<N; ++i)
    {
        out.points.push_back(scanner2World * points[i]);
        pc[i].getVector3fMap() = points[i].cast<float>();
    }
    _pointCloud_in_world.write(out);
    mls->mergePointCloud(pc, scanner2World);
    viz.updateData(*mls);


}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PointCloudAggregator.hpp for more detailed
// documentation about them.

bool PointCloudAggregator::configureHook()
{
    if (! PointCloudAggregatorBase::configureHook())
        return false;
    using namespace envire::maps;
    // TODO get values from config
    Eigen::Vector2d res(1, 1);
    Vector2ui numCells(50, 50);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    mls_config.gapSize = 0.25f;
    mls_config.useNegativeInformation = !true;
    mls.reset(new MLSGrid(numCells, res, mls_config));
    mls->getGrid().getLocalFrame().translation() << 0.5*mls->getGrid().getSize()+Eigen::Vector2d(0.5,0.5), 0;


    return true;
}
bool PointCloudAggregator::startHook()
{
    if (! PointCloudAggregatorBase::startHook())
        return false;


//    app.start();
//
//    //create vizkit3d plugin for showing envire
//    vizkit3d::MLSGridVisualization *mls_viz = new vizkit3d::MLSGridVisualization();
//    mls_viz->updateData(*mls);
//
//    //create vizkit3d widget
//    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
//    // grid plugin
//    vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
//    widget->addPlugin(grid_viz);
//    // add envire plugin
//    widget->addPlugin(mls_viz);

    return true;
}
void PointCloudAggregator::updateHook()
{
    PointCloudAggregatorBase::updateHook();
    return;
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    base::samples::RigidBodyState rbs;
    while(_bodyPose.read(rbs)== RTT::NewData)
    {
        std::cout << ".";
    }
    base::samples::Pointcloud pc;
    while(_pointCloud.read(pc)== RTT::NewData)
    {
        std::cout << "Pointcloud\n";
    }
}
void PointCloudAggregator::errorHook()
{
    PointCloudAggregatorBase::errorHook();
}
void PointCloudAggregator::stopHook()
{
    PointCloudAggregatorBase::stopHook();
}
void PointCloudAggregator::cleanupHook()
{
    PointCloudAggregatorBase::cleanupHook();
}
