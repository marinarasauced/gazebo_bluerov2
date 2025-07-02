
#include <math.h>

#include <gz/msgs/fluid_pressure.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "PressureSensor.hh"

using namespace custom;

//////////////////////////////////////////////////
bool PressureSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("pressure" != type)
  {
    gzerr << "Trying to load [pressure] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  gz::sensors::Sensor::Load(_sdf);

  this->pub = this->node.Advertise<gz::msgs::FluidPressure>(this->Topic());

  // Load noise if available
  if (_sdf.Element()->HasElement("noise"))
  {
    sdf::Noise noiseSdf;
    noiseSdf.Load(_sdf.Element()->GetElement("noise"));
    this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
    if (!this->noise)
    {
      gzerr << "Failed to load noise model from SDF.\n";
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool PressureSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  gz::msgs::FluidPressure msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);

  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  msg.set_pressure(this->pressure);
  // msg.set_variance(this->noise ? this->noise->Variance() : 0.0);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void PressureSensor::NewPosition(const gz::math::Vector3d &_pos)
{
  this->prevPos = _pos;

  // Constants
  const double rho = 1000.0; // kg/m^3
  const double g = 9.81;     // m/s^2
  const double P_atm = 101325.0; // Pa

  // Depth in meters (positive downward)
  double depth = -_pos.Z();
  if (depth < 0)
    depth = 0;

  double pressure = P_atm + rho * g * depth;

  // Add noise if configured
  if (this->noise)
    pressure = this->noise->Apply(pressure);

  this->pressure = pressure;
}

//////////////////////////////////////////////////
const gz::math::Vector3d &PressureSensor::Position() const
{
  return this->prevPos;
}

