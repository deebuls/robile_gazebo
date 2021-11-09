
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

#include "robile_gazebo/SimpleController.h"
#include "robile_gazebo/Robile.h"

Robile::Robile(ros::NodeHandle& nh):
_nh(nh),
_cmdVelX(0.0),
_cmdVelY(0.0),
_cmdVelA(0.0),
_initialized(false) {
    _jointStatesSubscriber = _nh.subscribe("/joint_states", 1, &Robile::jointStatesCallBack, this);
    _gazeboLinkStatesSubscriber = _nh.subscribe("/gazebo/link_states", 1, &Robile::gazeboLinkStatesCallBack, this);
    _odomPublisher = _nh.advertise<nav_msgs::Odometry>("/odom", 1);
    _odomTFPublisher = _nh.advertise<tf2_msgs::TFMessage>("/tf", 1);
    _pivotMarkersPublisher = _nh.advertise<visualization_msgs::MarkerArray>("/pivot_markers", 1);

    setOdomFrequency(10.0);
}

void Robile::setCmdVel(double vx, double vy, double va) {
    _cmdVelX = vx;
    _cmdVelY = vy;
    _cmdVelA = va;
}

void Robile::publishPivotMarkers() const {
    if (_pivotMarkersPublisher.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markerArray;
        int markerId = 0;
        for (const auto& drive: _drives) {
            visualization_msgs::Marker marker = drive.second.getPivotMarker();
            marker.id = markerId++;
            markerArray.markers.push_back(marker);
        }
        _pivotMarkersPublisher.publish(markerArray);
    }
}

void Robile::setOdomFrequency(double frequency) {
    _odomDuration = ros::Duration(1.0/frequency);
}

void Robile::step() {
    _controller.setPlatformTargetVelocity(_cmdVelX, _cmdVelY, _cmdVelA);
    _controller.calculatePlatformRampedVelocities();

    for (const auto& drive: _drives) {
        const std::string& driveName = drive.first;
        int wheelNumber = _wheelConfigs[driveName].ethercatNumber;
        float setpoint1, setpoint2;
        _controller.calculateWheelTargetVelocity(wheelNumber, drive.second.getPivotOrientation(), setpoint1, setpoint2);
        _drives.at(driveName).setHubWheelVelocities(setpoint1, setpoint2);
    }

/*
    // Example with simple controller
    std::map< std::string, std::pair< double, double > > controlCommands = 
        getWheelVelocities(_drives, _cmdVelX, _cmdVelY);

    for (const auto& drive: controlCommands) {
        const std::string& driveName = drive.first;
        const std::pair<double, double>& wheelVelocities = drive.second;
        _drives.at(driveName).setHubWheelVelocities(wheelVelocities.first, wheelVelocities.second);
    }
*/
}

void  Robile::initDrives(const std::map<std::string, double>& pivotJointData) {
    if (!_drives.empty())
        return;

    ros::Time now = ros::Time::now();
    for (auto& joint: pivotJointData) {
        std::string driveName = getKeloDriveName(joint.first);
        std::string pivotLink = std::string("/") + driveName + std::string("_drive_pivot_link");
        if (!_tfListener.waitForTransform("/base_link", pivotLink, now, ros::Duration(2.0))) {
            ROS_WARN("Did not receive transform from /base_link to %s", pivotLink.c_str());
            _drives.clear();
            return;
        }
        tf::StampedTransform transform;
        _tfListener.lookupTransform("/base_link", pivotLink, now, transform);

        _drives.insert(std::make_pair(driveName,
                                      KeloDrive(_nh,
                                                driveName, 
                                                transform.getOrigin().x(), 
                                                transform.getOrigin().y(), 
                                                transform.getOrigin().z(),
                                                joint.second)));
    }

    // init controller
    std::vector<kelo::WheelConfig> wheelConfigsVector;
	int wheelNumber = 0;
    double zDummy = 0;
    for (const auto& drive: _drives) {
        kelo::WheelConfig wc;
        wc.ethercatNumber = wheelNumber; // to associate between wheel name used in simulation and number used in controller
        drive.second.getPos(wc.x, wc.y, zDummy);
        wc.a = 0; // assume zero in simulation
        _wheelConfigs[drive.first] = wc;
        wheelConfigsVector.push_back(wc);
        wheelNumber++;
    }
    _controller.initialise(wheelConfigsVector);

    _initialized = true;
    ROS_INFO("Initialized %d Kelo drives", _drives.size());
}

void Robile::jointStatesCallBack(const sensor_msgs::JointState& msg) {
    const std::vector<std::string>& jointNames = msg.name;
    const std::vector<double>& jointPositions = msg.position;

    if (jointNames.empty() || jointNames.size() != jointPositions.size()) {
        return;
    }

    std::map<std::string, double> pivotJointData;
    std::string pivotJointNameIdentifier = "pivot_joint";
    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::string jointName = jointNames[i];
        if (jointName.find(pivotJointNameIdentifier) != std::string::npos) {
            // Normalize the pivot angle in range [0, 2*PI]
            double pivotAngle = jointPositions[i] - (int(jointPositions[i] / (2*M_PI)) * 2 * M_PI);
            pivotJointData[jointName] = pivotAngle;
        }
    }

    if (_drives.empty()) {
        initDrives(pivotJointData);
    } else {
        setPivotOrientations(pivotJointData);
    }
}

void Robile::gazeboLinkStatesCallBack(const gazebo_msgs::LinkStates& msg) {
    const std::vector<std::string>& linkNames = msg.name;
    const std::vector<geometry_msgs::Pose>& linkPoses = msg.pose;
    const std::vector<geometry_msgs::Twist>& linkTwist = msg.twist;

    for (size_t i = 0; i < linkNames.size(); ++i) {
        std::string name = linkNames[i];
        if (name.find("base_link") != std::string::npos) {
            _odomMsg.pose.pose = linkPoses[i];
            _odomMsg.twist.twist = linkTwist[i];
            break;
        }
    }
}

void Robile::setPivotOrientations(const std::map<std::string, double>& pivotJointData) {
    for (const auto& joint: pivotJointData) {
        std::string driveName = getKeloDriveName(joint.first);
        if (_drives.find(driveName) == _drives.end()) {
            ROS_ERROR("Cannot set pivot orientation for drive %s", driveName.c_str());
            continue;
        }
        KeloDrive& drive = _drives.at(driveName);
        drive.setPivotOrientation(joint.second);
    }
}

std::string Robile::getKeloDriveName(const std::string& jointName) {
    return jointName.substr(0, jointName.find("_drive_"));
}

void Robile::publishOdom() {
    ros::Time now = ros::Time::now();
    if (_initialized &&
        _odomPublisher.getNumSubscribers() > 0 &&
        now - _lastOdomPubTime >= _odomDuration) {
        _odomMsg.header.stamp = now;
        _odomMsg.header.frame_id = "odom";
        _odomMsg.child_frame_id = "base_link";
        _odomPublisher.publish(_odomMsg);
        _lastOdomPubTime = now;
    }
}

void Robile::publishOdomToBaseLinkTF() {
    if (_initialized) {
        tf2_msgs::TFMessage tfMsg;
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "odom";
        transform.header.stamp = ros::Time::now();
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = _odomMsg.pose.pose.position.x;
        transform.transform.translation.y = _odomMsg.pose.pose.position.y;
        transform.transform.translation.z = _odomMsg.pose.pose.position.z;
        transform.transform.rotation = _odomMsg.pose.pose.orientation;
        tfMsg.transforms.push_back(transform);

        _odomTFPublisher.publish(tfMsg);
    }
}
