// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ArduCopterDroneController_hpp
#define msr_airlib_ArduCopterDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/lidar/LidarBase.hpp"

#include "UdpSocket.hpp"

#include <sstream>
#include <iostream>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace msr { namespace airlib {

class ArduCopterApi : public MultirotorApiBase {

public:
    ArduCopterApi(const MultiRotorParams* vehicle_params, const AirSimSettings::MavLinkConnectionInfo& connection_info)
        : ip(connection_info.ip_address), vehicle_params_(vehicle_params)
    {
        connection_info_ = connection_info;
        sensors_ = &getSensors();

        connect(); // Should we try catching exceptions here?
    }

    ~ArduCopterApi()
    {
        closeConnections();
    }


public: 
    virtual void reset() override
    {
        MultirotorApiBase::reset();

        // Reset state
    }

    // Update sensor data & send to Ardupilot
    virtual void update() override
    {
        MultirotorApiBase::update();

        sendSensors();
        recvRotorControl();
    }

    // TODO:VehicleApiBase implementation
    virtual bool isApiControlEnabled() const override
    {
        return false;
    }
    virtual void enableApiControl(bool) override
    {
    }
    virtual bool armDisarm(bool) override
    {
        return false;
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }
    virtual void getStatusMessages(std::vector<std::string>&) override
    {
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //TODO:MultirotorApiBase implementation
    virtual real_T getActuation(unsigned int rotor_index) const override
    {
        return rotor_controls_[rotor_index];
    }
    virtual size_t getActuatorCount() const override
    {
        return vehicle_params_->getParams().rotor_count;
    }
    virtual void moveByRC(const RCData& rc_data) override
    {
        setRCData(rc_data);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State*, const Environment*) override
    {
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        last_rcData_ = rc_data;
        is_rc_connected = true;

        return true;
    }

protected: 
    virtual Kinematics::State getKinematicsEstimated() const override
    {
		Kinematics::State state;
		return state;
    }

    virtual Vector3r getPosition() const override
    {
        return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual Vector3r getVelocity() const override
    {
		return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual Quaternionr getOrientation() const override
    {
        return Quaternionr(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual LandedState getLandedState() const override
    {
		return LandedState::Landed;
    }

    virtual RCData getRCData() const override
    {
        //return what we received last time through setRCData
        return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
		return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        // pick a number, 3 meters is probably safe 
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        // return params_.takeoff.takeoff_z;
        return 3.0;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

    virtual void commandRollPitchThrottle(float pitch, float roll, float throttle, float yaw_rate) override
    {
        Utils::log(Utils::stringf("commandRollPitchThrottle %f, %f, %f, %f", pitch, roll, throttle, yaw_rate));
    }

    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        Utils::log(Utils::stringf("commandRollPitchZ %f, %f, %f, %f", pitch, roll, z, yaw));
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

protected:
    void closeConnections()
    {
        if (udpSocket_ != nullptr)
            udpSocket_->close();
    }

    void connect()
    {
        port = static_cast<uint16_t>(connection_info_.ip_port);

        closeConnections();

        if (ip == "") {
            throw std::invalid_argument("UdpIp setting is invalid.");
        }

        if (port == 0) {
            throw std::invalid_argument("UdpPort setting has an invalid value.");
        }

        Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.sitl_ip_port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);

        udpSocket_ = std::make_shared<mavlinkcom::UdpSocket>();
        udpSocket_->bind(connection_info_.local_host_ip, connection_info_.sitl_ip_port);
    }

    const GpsBase* getGps() const
    {
        return static_cast<const GpsBase*>(sensors_->getByType(SensorBase::SensorType::Gps));
    }
    const ImuBase* getImu() const
    {
        return static_cast<const ImuBase*>(sensors_->getByType(SensorBase::SensorType::Imu));
    }
    const MagnetometerBase* getMagnetometer() const
    {
        return static_cast<const MagnetometerBase*>(sensors_->getByType(SensorBase::SensorType::Magnetometer));
    }
    const BarometerBase* getBarometer() const
    {
        return static_cast<const BarometerBase*>(sensors_->getByType(SensorBase::SensorType::Barometer));
    }
    const LidarBase* getLidar() const
    {
        return static_cast<const LidarBase*>(sensors_->getByType(SensorBase::SensorType::Lidar));
    }

private:
    //convert pitch, roll, yaw from -1 to 1 to PWM
    static uint16_t angleToPwm(float angle)
    {
        return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
    }
    static uint16_t thrustToPwm(float thrust)
    {
        return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
    }
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

    Vector3r bodyAnglesToEarthAngles(Vector3r bodyRPY, Vector3r bodyVelocityRPY)
    {
        float p = bodyVelocityRPY[0];
        float q = bodyVelocityRPY[1];
        float r = bodyVelocityRPY[2];

        // Roll, pitch, yaw
        float phi = bodyRPY[0];
        float theta = bodyRPY[1];

        float phiDot = p + tan(theta)*(q*sin(phi) + r * cos(phi));

        float thetaDot = q * cos(phi) - r * sin(phi);
        if (fabs(cos(theta)) < 1.0e-20)
        {
            theta += 1.0e-10f;
        }

        float psiDot = (q*sin(phi) + r * cos(phi)) / cos(theta);

        return Vector3r(phiDot, thetaDot, psiDot);
    }

    virtual void normalizeRotorControls()
    {
        // change 1000-2000 to 0-1.
        for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
            rotor_controls_[i] = (rotor_controls_[i] - 1000.0f) / 1000.0f;
        }
    }

    void sendSensors()
    {
        if (sensors_ == nullptr)
            return;

        const auto& gps_output = getGps()->getOutput();
        const auto& imu_output = getImu()->getOutput();
        // const auto& baro_output = getBarometer()->getOutput();
        // const auto& mag_output = getMagnetometer()->getOutput();

        std::ostringstream oss;

        // Send RC channels to Ardupilot if present
        if (is_rc_connected && last_rcData_.is_valid) {
            oss << ","
                   "\"rc\": {"
                   "\"channels\": ["
                << (last_rcData_.roll + 1) * 0.5f << ","
                << (last_rcData_.yaw + 1) * 0.5f << ","
                << (last_rcData_.throttle + 1) * 0.5f << ","
                << (-last_rcData_.pitch + 1) * 0.5f << ","
                << static_cast<float>(last_rcData_.getSwitch(0)) << ","
                << static_cast<float>(last_rcData_.getSwitch(1)) << ","
                << static_cast<float>(last_rcData_.getSwitch(2)) << ","
                << static_cast<float>(last_rcData_.getSwitch(3))
                << "]}";
        }

        const auto lidar = getLidar();
        // TODO: Add bool value in settings to check whether to send lidar data or not
		// Since it's possible that we don't want to send the lidar data to Ardupilot but still have the lidar (maybe as a ROS topic)
        if (lidar != nullptr) {
            const auto& lidar_output = lidar->getOutput();
            oss << ","
                   "\"lidar\": {"
                   "\"point_cloud\": [";

            std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(oss, ","));
            oss << "]}";
        }

        float yaw;
        float pitch;
        float roll;
        VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

        char buf[65000];

		// TODO: Split the following sensor packet formation into different parts for individual sensors

        // UDP packets have a maximum size limit of 65kB
        int ret = snprintf(buf, sizeof(buf),
                           "{"
                           "\"timestamp\": %" PRIu64 ","
                           "\"imu\": {" 
                           "\"angular_velocity\": [%lf, %lf, %lf],"
                           "\"linear_acceleration\": [%lf, %lf, %lf]"
                           "},"
                           "\"gps\": {"
                           "\"lat\": %lf,"
                           "\"lon\": %lf,"
                           "\"alt\": %lf"
                           "},"
                           "\"velocity\": {"
                           "\"world_linear_velocity\": [%lf, %lf, %lf]"
                           "},"
                           "\"pose\": {"
                           "\"roll\": %lf,"
                           "\"pitch\": %lf,"
                           "\"yaw\": %lf"
                           "}"
                           "%s"
                           "}\n",
                           static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E3),
                           imu_output.angular_velocity[0],
                           imu_output.angular_velocity[1],
                           imu_output.angular_velocity[2],
                           imu_output.linear_acceleration[0],
                           imu_output.linear_acceleration[1],
                           imu_output.linear_acceleration[2],
                           gps_output.gnss.geo_point.latitude,
                           gps_output.gnss.geo_point.longitude,
                           gps_output.gnss.geo_point.altitude,
                           gps_output.gnss.velocity[0],
                           gps_output.gnss.velocity[1],
                           gps_output.gnss.velocity[2],
                           roll, pitch, yaw,
                           oss.str().c_str());

		if (ret < 0) {
			Utils::log("Encoding error while forming sensor message", Utils::kLogLevelInfo);
			return;
		}
        else if (static_cast<uint>(ret) >= sizeof(buf)) {
            Utils::log(Utils::stringf("Sensor message truncated, lost %d bytes", ret - sizeof(buf)), Utils::kLogLevelInfo);
        }

        // Send data
        if (udpSocket_ != nullptr) {
            udpSocket_->sendto(buf, strlen(buf), ip, port);
        }
    }

    void recvRotorControl()
    {
        // Receive motor data
        RotorControlMessage pkt;
        int recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        while (recv_ret != sizeof(pkt)) {
            if (recv_ret <= 0) {
                Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
            } else {
                Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(pkt)), Utils::kLogLevelInfo);
            }
            
            recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        }

        for (auto i = 0; i < RotorControlCount && i < kArduCopterRotorControlCount; ++i) {
            rotor_controls_[i] = pkt.pwm[i];
        }

        normalizeRotorControls();
    }

private:
    static const int kArduCopterRotorControlCount = 11;

    struct RotorControlMessage {
        uint16_t pwm[kArduCopterRotorControlCount];
    };

    std::shared_ptr<mavlinkcom::UdpSocket> udpSocket_;

    AirSimSettings::MavLinkConnectionInfo connection_info_;
    uint16_t port;
    const std::string& ip;
    const SensorCollection* sensors_;
    const MultiRotorParams* vehicle_params_;

    MultirotorApiParams safety_params_;

    RCData last_rcData_;
    bool is_rc_connected;

    // TODO: Increase to 6 or 8 for hexa or larger frames, 11 was used in SoloAPI
    static const int RotorControlCount = 4;

    float rotor_controls_[RotorControlCount];
};

}} //namespace
#endif
