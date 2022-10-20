#ifndef FLATLAND_PLUGINS_IKA_INCLUDE_SIMULATED_TIRE_H_
#define FLATLAND_PLUGINS_IKA_INCLUDE_SIMULATED_TIRE_H_

#include <Box2D/Box2D.h>

/**
 * @brief A simulated tire for the box 2d simulation framework.
 * @details This tire will do a force and impulse based simulation of a real tire in a top down 2D environment.
 */
class SimulatedTire {
 private:
  /**
   * @brief Short access to the tire body of Box2D. [not owning]
   */
  b2Body *tire_physics_body_ = nullptr;

  /**
   * @brief Parameter that clamps the maximum acceleration of the tire. [m/s^s]
   */
  double maximum_acceleration_ = 0.0;

  /**
   * @brief Parameter that clamps the maximum deceleration of the tire. [m/s^2]
   */
  double maximum_deceleration_ = 0.0;

  /**
   * @brief Parameter that clamps the lateral impulse to a maximum value. [Ns]
   * @details This value will only be reached at maximum_velocity_.
   */
  double maximum_lateral_impulse_ = 0.0;

  /**
   * @brief Parameter that available lateral impulse.
   * @details That maximum_lateral_impulse_ will be produced at maximum
   * velocity. For all other velocities the impulse will be scaled.
   */
  double maximum_velocity_ = 0.0;

  /**
   * @brief Multiplier for the current traction to the ground.
   * @details Can be used to simulate different grounds.
   */
  double traction_multiplier_ = 1.0;

  /**
   * @brief Multiplier for the current drag to the air.
   * @details Can be used so simulate wind or tunnels.
   */
  double drag_multiplier_ = 1.0;

 public:

  /**
   * @brief Will initialise this tire by setting the parameter.
   * @param tire_physics_body Box2D body of the tire.
   * @param maximum_acceleration Max acceleration of the wheel.
   * @param maximum_deceleration Max deceleration of the wheel.
   * @param maximum_velocity Max velocity of the wheel.
   * @param maximum_lateral_impulse Max lateral impulse of the wheel.
   */
  void Initialise(b2Body *tire_physics_body,
                  double maximum_acceleration,
                  double maximum_deceleration,
                  double maximum_velocity,
                  double maximum_lateral_impulse);

  /**
   * @brief Will calculate the current lateral velocity.
   * @details Positive to the right.
   * @return Lateral velocity.
   */
  b2Vec2 GetLateralVelocity();

  /**
   * @brief Will return the current forward velocity.
   * @details Positive to the front.
   * @return Forward velocity.
   */
  b2Vec2 GetForwardVelocity();

  /**
   * @brief Will update the loss and driving forces.
   * @param desired_acceleration
   * @param step_size
   */
  void UpdateTire(double desired_acceleration, double step_size);

  /**
   * @brief Will update the loss values due to driving behaviour and
   * environment.
   */
  void UpdateDragAndFrictionLoss();

  /**
   * @breif Will calculate the new forces to reach the desired state.
   * @param desired_acceleration Desired acceleration.
   * @param step_size Step time of this integration.
   */
  void UpdateDriveForces(double desired_acceleration, double step_size);
};

#endif //FLATLAND_PLUGINS_IKA_INCLUDE_SIMULATED_TIRE_H_
