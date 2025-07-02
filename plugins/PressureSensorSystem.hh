#ifndef PRESSURE_SENSOR_SYSTEM_HH_
#define PRESSURE_SENSOR_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <unordered_map>

#include "PressureSensor.hh" 

namespace custom
{
  /// \brief System plugin to manage a custom pressure sensor
  class PressureSensorSystem:
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) final;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) final;

    private: void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

    /// \brief Map of simulation entities to custom pressure sensors
    private: std::unordered_map<gz::sim::Entity,
                                std::shared_ptr<PressureSensor>> entitySensorMap;

    /// \brief GZ Transport node for publishing
    private: gz::transport::Node node;
  };
}

#endif
