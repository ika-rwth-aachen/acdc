#ifndef FLATLAND_IKA_PLUGINS_LASER_CALLBACK_H
#define FLATLAND_IKA_PLUGINS_LASER_CALLBACK_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>

#include "sensors/LaserScanner.h"

namespace flatland_ika_plugins {

/**
 * This class handles the b2RayCastCallback ReportFixture method
 * allowing each thread to access its own callback object
 */
class LaserCallback : public b2RayCastCallback {
 private:
  bool hit_ = false; ///< Box2D ray trace checking if ray hits anything
  double distance_; ///< Box2D ray trace fraction
  double intensity_; ///< Intensity of ray trace collision
  double angle_ = 0.0; ///< Angle of ray
  tf::Point point_ = {0.0, 0.0, 0.0}; ///< Point there the laser hit

  b2Body *physics_body_ = nullptr; ///< Phisyscs body that was hit
  flatland_server::Body *body_ = nullptr; ///< Body that was hit
  LaserScanner *parent_ = nullptr; ///< The parent Laser plugin

 public:

  /**
   * @brief Basic contructor that sets the parent.
   * @param parent Laser scanner that utilises this callback.
   */
  LaserCallback(LaserScanner *parent, double angle);

  /**
   * @brief Box2D raytrace call back method required for implementing the
   * b2RayCastCallback abstract class
   * @param[in] fixture Fixture the ray hits
   * @param[in] point Point the ray hits the fixture
   * @param[in] normal Vector indicating the normal at the point hit
   * @param[in] fraction Fraction of ray length at hit point
   */
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override;

  bool hasHit() const;

  double getDistance() const;

  double getIntensity() const;

  double getAngle() const;

  b2Body *getPhysicsBody() const;

  flatland_server::Body *getBody() const;

  tf::Point getPoint() const;

};
}
#endif // FLATLAND_IKA_PLUGINS_LASER_CALLBACK_H
