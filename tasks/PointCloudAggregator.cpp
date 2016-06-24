/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointCloudAggregator.hpp"

//#include <vizkit3d/Vizkit3DWidget.hpp>
//#include "MLSMapVisualization.hpp"
//#include <vizkit3d/GridVisualization.hpp>


using namespace ::maps;

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
    ::maps::grid::PointCloud pc;
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
    using namespace ::maps::grid;
    // TODO get values from config
    Eigen::Vector2d res(0.1, 0.1);
    Vector2ui numCells(500, 500);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    mls_config.gapSize = 0.25f;
    mls_config.useNegativeInformation = !true;
    mls.reset(new MLSMapSloped(numCells, res, mls_config));
    mls->getLocalFrame().translation() << 0.5*mls->getSize()+Eigen::Vector2d(0.5,0.5), 0;


    return true;
}
bool PointCloudAggregator::startHook()
{
    if (! PointCloudAggregatorBase::startHook())
        return false;


//    app.start();
//
//    //create vizkit3d plugin 
//    vizkit3d::MLSMapVisualization *mls_viz = new vizkit3d::MLSMapVisualization();
//    mls_viz->updateData(*mls);
//
//    //create vizkit3d widget
//    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
//    // grid plugin
//    vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
//    widget->addPlugin(grid_viz);
//    // add plugin
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
    std::cout << "stopHook, do serialization\n";
    std::ofstream stream("output_mls.bin", std::ios::binary);
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << *mls;

    PointCloudAggregatorBase::stopHook();
}
void PointCloudAggregator::cleanupHook()
{
    PointCloudAggregatorBase::cleanupHook();
}
