/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <definitions/IkaObjectList.h>
#include <definitions/utility/object_definitions.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "IkaObjectVisualSelectionHandler.h"

#define MIN_OPACITY 0.3
#define MAX_OPACITY 1.0

namespace rviz_plugin {

/**
 * @authors Simon Schaefer
 * @date 10.12.2018
 * @brief Actually visible representation of one object inside an IkaObjectList.
 * @details This will use basic rviz shapes to visualize one instance of an IkaObject.
 * Currently visualised quantities are:
 * - Position
 * - Velocity
 * - Classification Type
 */
class IkaObjectVisual {
 private:
  Ogre::ColourValue geometric_color; /**< Color of all geometric objects. **/
  Ogre::ColourValue text_color; /**< Color of all text elements. **/

  std::shared_ptr<rviz::Arrow> velocity_arrow_ = nullptr; /**< Arrow to visualise the velocity. **/
  std::shared_ptr<rviz::Shape> body_box_ = nullptr; /**< Box to visualise the size and position. **/
  std::shared_ptr<rviz::MovableText> type_name_text_ = nullptr; /**< Text field to visualise the typename. **/

  Ogre::SceneNode *frame_node_ = nullptr; /**< Frame to move the frame of reference. **/
  Ogre::SceneNode *text_node_ = nullptr; /**< Frame to move the text field in relative relation to the frame of reference. **/

  float z_shift_ = 0.0f; /**< Shift for geometric objects. **/
  float text_shift_ = 0.0f; /**< Shift for text elements. **/

  Ogre::SceneManager *scene_manager_ = nullptr; /** Ogre manager for the 3D scene. **/

  std::shared_ptr<rviz_plugin::IkaObjectVisualSelectionHandler> selection_handler_;

  definitions::IkaObject object;

  /**
   * @brief Will map from real existence probabilities to display able once.
   * @details Alpha values that are to low ca not be seen correctly and to high once are to opaque
   * to see trough. This function will make a linear interpolation.
   * @param real_existence_probability The existence probability between 0.0 and 1.0.
   * @return Display able alpha value between MIN_OPACITY and MAX_OPACITY.
   */
  float calculateDisplayExistenceProbability(float real_existence_probability);

  void generateObjectInfoString(
      std::string& msg_str,
      const definitions::IkaObject& msg);

 public:
  /**
   * @brief Basic constructor for initialisation of an 3D object.
   * @details The scene manager is the global Ogre manager for the hole 3D scene.
   * The parent node is the node were all objects in this visualisation have to be
   * appended. The node parent, child will create a inheritance tree for frames.
   * @param scene_manager Global 3D scene manager.
   * @param parent_node Parent node of this object.
   * @param context Rviz display context
   */
  IkaObjectVisual(Ogre::SceneManager *scene_manager,
                  Ogre::SceneNode *parent_node,
                  rviz::DisplayContext *context);

  /**
   * @brief Basic destructor to delete all children nodes.
   * @details Will use the ogre manager to delete frames.
   */
  virtual ~IkaObjectVisual();

  /**
   * @brief Will send new data into this visualisation object.
   * @details All information contained in the IkaObject will be converted into a visual representation.
   * @param msg IkaObject that should be visualised.
   */
  void setMessage(const definitions::IkaObject &msg, bool visible);

  /**
   * @brief Will set the position of the Frame of reference.
   * @details The Frame of reference should be provided by the state publisher and a corresponding "urdf" file.
   * @param position New position for the frame of reference.
   */
  void setFramePosition(const Ogre::Vector3 &position);
  /**
   * @brief Will set the orientation of the Frame of reference.
   * @details The Frame of reference should be provided by the state publisher and a corresponding "urdf" file.
   * @param orientation New orientation for the frame of reference.
   */
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  /**
   * @brief Will set the color of all geometric objects.
   * @details This function only effects the geometric objects. Texts will not be effected.
   * @param color New color.
   */
  void setObjectColor(Ogre::ColourValue color);

  /**
   * @brief Will set the color of all text elements.
   * @details This function only effects the text elements. Objects will not be effected.
   * @param color New color.
   */
  void setTextColor(Ogre::ColourValue color);

  /**
   * @brief Will set the geometric shift for all objects.
   * @details To avoid flickering inside of 3D scene all objects will be shifted about some cm.
   * @param z_shift Value to shift.
   */
  void setZShift(float z_shift);

  /**
   * @brief Will set the text shift for all elements.
   * @details To avoid flickering inside of 3D scene all elemnts will be shifted about some cm.
   * @param text_shift Value to shift.
   */
  void setTextShift(float text_shift);
};

}  // namespace rviz_plugin
