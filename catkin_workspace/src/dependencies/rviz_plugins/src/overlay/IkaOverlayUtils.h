// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/Overlay/OgrePanelOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreOverlayManager.h>

#include <QImage>
#include <QColor>

/**
 * Move the copied classes to out namespace.
 */
namespace rviz_plugin {
class OverlayObject;

/**
 * This class is copied from project jsk_rviz_plugins to avoid installation process.
 */
class ScopedPixelBuffer {
 public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject &overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor &bg_color);
  virtual QImage getQImage(OverlayObject &overlay, QColor &bg_color);
 protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
 private:

};

/**
 * This class is copied from project jsk_rviz_plugins to avoid installation process.
 */
class OverlayObject {
 public:
  explicit OverlayObject(const std::string &name);
  virtual ~OverlayObject();

  virtual std::string getName();
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady();
  virtual bool updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(double left, double top);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible();
  virtual unsigned int getTextureWidth();
  virtual unsigned int getTextureHeight();
 protected:
  const std::string name_;
  Ogre::Overlay *overlay_;
  Ogre::PanelOverlayElement *panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;

 private:

};

}  // namespace rviz_plugin
