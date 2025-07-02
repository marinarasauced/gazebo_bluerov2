
#ifndef PRESSURE_SENSOR_HH_
#define PRESSURE_SENSOR_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>

namespace custom
{
  /// \brief A simulated pressure sensor that calculates hydrostatic pressure
  /// based on the Z (depth) position in water, with optional noise.
  class PressureSensor : public gz::sensors::Sensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and publish pressure data.
    /// \param[in] _now The current time.
    /// \return True if the update was successful.
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Provide the current position of the robot to compute depth-based pressure.
    /// \param[in] _pos Current position in world coordinates.
    public: void NewPosition(const gz::math::Vector3d &_pos);

    /// \brief Get the most recently stored world position.
    /// \return The latest position provided to the sensor.
    public: const gz::math::Vector3d &Position() const;

    /// \brief Previous position of the robot.
    private: gz::math::Vector3d prevPos{std::nan(""), std::nan(""), std::nan("")};

    /// \brief Computed pressure in Pascals.
    private: double pressure{0.0};

    /// \brief Noise model applied to the pressure data.
    private: gz::sensors::NoisePtr noise{nullptr};

    /// \brief Transport node for communication.
    private: gz::transport::Node node;

    /// \brief Publisher for the pressure sensor data.
    private: gz::transport::Node::Publisher pub;
  };
}

#endif