require 'vizkit'
require 'orocos'
require 'orocos/log'
require "transformer/runtime"
require 'rock/bundle'
include Orocos

Orocos::CORBA.max_message_size = 80000000

puts ARGV[0]

if not ARGV[0]
    puts "add a valid logfile folder as parameter"
    exit
end

Bundles.initialize



Bundles.run \
            'envire_maps::PointCloudAggregator' => 'pc_agg',
            'output' => nil do
    log = Orocos::Log::Replay.open(ARGV)
    log.use_sample_time = false

    pc_agg = TaskContext.get("pc_agg")
    # markerDetection = TaskContext.get("markerDetection")

#    Orocos.conf.apply( calib, ['default'] )
    pc_agg.apply_conf_file('ConfigFiles/config.yml',['simulator'])
    pc_agg.configure

    #    Orocos.conf.apply( markerDetection, ['default'] )

#    log.usbcam_tower_left.frame.connect_to markerDetection.image
    #log.camera_front_left.frame.connect_to markerDetection.image
    #log.usbcam_tower_left.frame.connect_to(markerDetection.image, :type => :buffer, :size => 10)
    log.velodyne.pointcloud.connect_to(pc_agg.pointCloud, :type => :buffer, :size => 10)
    log.xsens.pose_samples.connect_to(pc_agg.bodyPose, :type => :buffer, :size => 10000)

#    Orocos.transformer.setup(calib)
#    Orocos.transformer.setup(markerDetection)


    pc_agg.configure()

    pc_agg.start()
    
    Vizkit.control log
#    Vizkit.display markerDetection.output_image
    Vizkit.exec
end
