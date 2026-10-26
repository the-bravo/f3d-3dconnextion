/**
 * @class   spacemouse_impl
 * @brief   3Dconnexion SpaceMouse navigation
 *
 * Connects the render window to the 3Dconnexion Navigation Library (navlib), which ships
 * with the 3DxWare driver. The driver computes the camera motion; this class answers its
 * questions about the camera, the model and the pointer, and applies the camera it sends back.
 * This is the same approach 3Dconnexion's own viewer and CAD programs use.
 *
 * The navlib calls back on the thread running the window message loop, so all methods
 * run on the main thread. Only built when F3D_MODULE_SPACEMOUSE is enabled.
 *
 * 3D input device development tools and related technology are provided under license
 * from 3Dconnexion. (c) 3Dconnexion 1992 - 2025. All rights reserved.
 */

#ifndef f3d_spacemouse_impl_h
#define f3d_spacemouse_impl_h

#include <memory>

namespace f3d::detail
{
class interactor_impl;
class window_impl;

class spacemouse_impl
{
public:
  spacemouse_impl(window_impl& window, interactor_impl& interactor);
  ~spacemouse_impl();

  spacemouse_impl(const spacemouse_impl&) = delete;
  spacemouse_impl& operator=(const spacemouse_impl&) = delete;

  /**
   * Open the connection to the 3DxWare driver.
   * Returns false, and logs why at debug level, when no driver is available.
   */
  bool Enable();

  /**
   * Close the connection to the 3DxWare driver.
   */
  void Disable();

  /**
   * Called on every event loop tick. Tells the driver when the model bounds
   * or the projection changed, for example after loading a file.
   */
  void Tick();

private:
  class internals;
  std::unique_ptr<internals> Internals;
};
}

#endif
