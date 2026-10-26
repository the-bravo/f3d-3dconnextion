// 3Dconnexion SpaceMouse navigation through the 3Dconnexion Navigation Library.
//
// 3D input device development tools and related technology are provided under license
// from 3Dconnexion. (c) 3Dconnexion 1992 - 2025. All rights reserved.
//
// Matrices exchanged with the navlib use column vectors stored row by row, the same
// layout as vtkMatrix4x4, so they can be copied element by element.

#include "spacemouse_impl.h"

#include "interactor_impl.h"
#include "log.h"
#include "window_impl.h"

#include "vtkF3DRenderer.h"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4201) // nameless struct/union in navlib types
#endif
#include <SpaceMouse/CNavigation3D.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <system_error>

namespace f3d::detail
{
namespace
{
using navlib::make_result_code;
namespace navlib_errc = navlib::navlib_errc;

long NoData()
{
  return make_result_code(navlib_errc::no_data_available);
}
long InvalidOperation()
{
  return make_result_code(navlib_errc::invalid_operation);
}
long NotSupported()
{
  return make_result_code(navlib_errc::function_not_supported);
}

// Name shown in the 3Dconnexion settings for F3D's button and speed profile
constexpr const char* ProfileName = "F3D";

// Pivot marker size as a fraction of the visible view height
constexpr double PivotMarkerScale = 0.012;
}

//----------------------------------------------------------------------------
class spacemouse_impl::internals : public TDx::SpaceMouse::Navigation3D::CNavigation3D
{
  using nav3d = TDx::SpaceMouse::Navigation3D::CNavigation3D;

public:
  internals(window_impl& window, interactor_impl& interactor)
    // Single threaded (callbacks arrive through the window message loop), column vectors
    : nav3d(false, true)
    , Window(window)
    , Interactor(interactor)
  {
    this->Picker->SetTolerance(0.005);
    this->Picker->PickFromListOff();

    // Pivot marker: a small orange sphere shown while the driver rotates around it
    this->PivotSource->SetThetaResolution(24);
    this->PivotSource->SetPhiResolution(16);
    this->PivotMapper->SetInputConnection(this->PivotSource->GetOutputPort());
    this->PivotActor->SetMapper(this->PivotMapper);
    this->PivotActor->GetProperty()->SetColor(1.0, 0.55, 0.1);
    this->PivotActor->GetProperty()->LightingOff();
    this->PivotActor->PickableOff();
    this->PivotActor->UseBoundsOff();
    this->PivotActor->VisibilityOff();
  }

  ~internals() override
  {
    this->Disable();
    if (vtkRenderer* ren = this->Renderer())
    {
      ren->RemoveActor(this->PivotActor);
    }
  }

  bool Enable()
  {
    try
    {
      nav3d::PutProfileHint(ProfileName);
      nav3d::EnableNavigation(true);
      // The driver paces the motion frames; F3D renders at the end of each one
      nav3d::PutFrameTimingSource(TimingSource::SpaceMouse);
    }
    catch (const std::exception& ex)
    {
      log::debug("SpaceMouse: not available (", ex.what(), ")");
      return false;
    }
    log::debug("SpaceMouse: connected to the 3Dconnexion driver");
    this->Tick();
    return true;
  }

  void Disable()
  {
    std::error_code ec;
    nav3d::EnableNavigation(false, ec);
  }

  void Tick()
  {
    if (!nav3d::IsEnabled())
    {
      return;
    }

    // Keep the pivot marker in the scene, a file reload may have cleared the renderer
    vtkRenderer* ren = this->Renderer();
    if (ren && !ren->HasViewProp(this->PivotActor))
    {
      ren->AddActor(this->PivotActor);
    }

    // Tell the driver about new model bounds, e.g. after a file was loaded
    navlib::box_t extents;
    if (this->GetModelExtents(extents) == 0)
    {
      std::array<double, 6> b = { extents.min.x, extents.max.x, extents.min.y, extents.max.y,
        extents.min.z, extents.max.z };
      if (b != this->LastBounds)
      {
        this->LastBounds = b;
        nav3d::Write(navlib::model_extents_k, extents);

        navlib::matrix_t front;
        if (this->GetFrontView(front) == 0)
        {
          nav3d::Write(navlib::views_front_k, front);
        }
      }
    }

    // Tell the driver when the projection is toggled between perspective and parallel
    if (vtkCamera* cam = this->Camera())
    {
      const bool perspective = !cam->GetParallelProjection();
      if (perspective != this->LastPerspective)
      {
        this->LastPerspective = perspective;
        nav3d::Write(navlib::view_perspective_k, perspective);
      }
    }
  }

protected:
  //--------------------------------------------------------------------------
  // Helpers
  vtkRenderer* Renderer() const
  {
    return this->Window.GetRenderer();
  }

  vtkCamera* Camera() const
  {
    vtkRenderer* ren = this->Renderer();
    return ren ? ren->GetActiveCamera() : nullptr;
  }

  double AspectRatio() const
  {
    const int* size = this->Window.GetRenderWindow()->GetSize();
    return size[1] > 0 ? static_cast<double>(size[0]) / size[1] : 1.0;
  }

  // Height of the visible area at the focal point, in world units
  double VisibleHeightAtFocalPoint() const
  {
    vtkCamera* cam = this->Camera();
    if (cam->GetParallelProjection())
    {
      return 2.0 * cam->GetParallelScale();
    }
    return 2.0 * cam->GetDistance() * std::tan(vtkMath::RadiansFromDegrees(cam->GetViewAngle()) / 2.0);
  }

  void UpdatePivotMarker()
  {
    if (!this->PivotActor->GetVisibility() || !this->Camera())
    {
      return;
    }
    this->PivotSource->SetCenter(this->Pivot.data());
    this->PivotSource->SetRadius(PivotMarkerScale * this->VisibleHeightAtFocalPoint());
  }

  //--------------------------------------------------------------------------
  // ISpace3D: which way is up and where is the front
  long GetCoordinateSystem(navlib::matrix_t& matrix) const override
  {
    // Rows are the navlib axes (X right, Y up, Z out of the screen) in world coordinates
    const auto [r, u, o] = this->SceneAxes();
    matrix = { r[0], r[1], r[2], 0, u[0], u[1], u[2], 0, o[0], o[1], o[2], 0, 0, 0, 0, 1 };
    return 0;
  }

  long GetFrontView(navlib::matrix_t& matrix) const override
  {
    // Camera to world transform of the front view: camera X right, Y up, looking along -out
    const auto [r, u, o] = this->SceneAxes();
    matrix = { r[0], u[0], o[0], 0, r[1], u[1], o[1], 0, r[2], u[2], o[2], 0, 0, 0, 0, 1 };
    return 0;
  }

  //--------------------------------------------------------------------------
  // IView: the camera
  long GetCameraMatrix(navlib::matrix_t& matrix) const override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    // VTK gives world to camera, the navlib wants camera to world
    double cameraToWorld[16];
    vtkMatrix4x4::Invert(cam->GetModelViewTransformMatrix()->GetData(), cameraToWorld);
    for (size_t i = 0; i < 16; i++)
    {
      matrix[i] = cameraToWorld[i];
    }
    return 0;
  }

  long SetCameraMatrix(const navlib::matrix_t& m) override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    const double eye[3] = { m[3], m[7], m[11] };
    const double up[3] = { m[1], m[5], m[9] };
    const double look[3] = { -m[2], -m[6], -m[10] };

    // Keep the focal distance so normal mouse rotation still orbits a sensible point
    const double dist = cam->GetDistance();
    const double focal[3] = { eye[0] + dist * look[0], eye[1] + dist * look[1],
      eye[2] + dist * look[2] };

    cam->SetPosition(eye);
    cam->SetFocalPoint(focal);
    cam->SetViewUp(up);
    this->CameraChanged = true;
    return 0;
  }

  long GetIsViewPerspective(navlib::bool_t& perspective) const override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    perspective = !cam->GetParallelProjection();
    return 0;
  }

  long GetViewFOV(double& fov) const override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    fov = vtkMath::RadiansFromDegrees(cam->GetViewAngle());
    return 0;
  }

  long SetViewFOV(double fov) override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    cam->SetViewAngle(vtkMath::DegreesFromRadians(fov));
    this->CameraChanged = true;
    return 0;
  }

  long GetViewFrustum(navlib::frustum_t& frustum) const override
  {
    vtkCamera* cam = this->Camera();
    if (!cam || cam->GetParallelProjection())
    {
      return InvalidOperation();
    }
    double range[2];
    cam->GetClippingRange(range);
    const double top = range[0] * std::tan(vtkMath::RadiansFromDegrees(cam->GetViewAngle()) / 2.0);
    const double right = top * this->AspectRatio();
    frustum.left = -right;
    frustum.right = right;
    frustum.bottom = -top;
    frustum.top = top;
    frustum.nearVal = range[0];
    frustum.farVal = range[1];
    return 0;
  }

  long SetViewFrustum(const navlib::frustum_t&) override
  {
    return NotSupported();
  }

  long GetViewExtents(navlib::box_t& extents) const override
  {
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    const double halfHeight = cam->GetParallelScale();
    const double halfWidth = halfHeight * this->AspectRatio();
    double range[2];
    cam->GetClippingRange(range);
    extents.min = { -halfWidth, -halfHeight, -range[1] };
    extents.max = { halfWidth, halfHeight, range[1] };
    return 0;
  }

  long SetViewExtents(const navlib::box_t& extents) override
  {
    // Zoom in parallel projection
    vtkCamera* cam = this->Camera();
    if (!cam)
    {
      return InvalidOperation();
    }
    const double halfHeight = (extents.max.y - extents.min.y) / 2.0;
    if (halfHeight > 0)
    {
      cam->SetParallelScale(halfHeight);
      this->CameraChanged = true;
    }
    return 0;
  }

  long GetPointerPosition(navlib::point_t& position) const override
  {
    vtkRenderer* ren = this->Renderer();
    HWND hwnd = static_cast<HWND>(this->Window.GetRenderWindow()->GetGenericWindowId());
    POINT pt;
    if (!ren || !hwnd || !::GetCursorPos(&pt) || !::ScreenToClient(hwnd, &pt))
    {
      return InvalidOperation();
    }
    // Windows counts pixels from the top, VTK from the bottom
    const int* size = this->Window.GetRenderWindow()->GetSize();
    ren->SetDisplayPoint(pt.x, size[1] - 1 - pt.y, 0.0); // 0 = on the near plane
    ren->DisplayToWorld();
    double world[4];
    ren->GetWorldPoint(world);
    if (world[3] == 0.0)
    {
      return NoData();
    }
    position = { world[0] / world[3], world[1] / world[3], world[2] / world[3] };
    return 0;
  }

  //--------------------------------------------------------------------------
  // IModel: what is in the scene
  long GetModelExtents(navlib::box_t& extents) const override
  {
    vtkRenderer* ren = this->Renderer();
    if (!ren)
    {
      return InvalidOperation();
    }
    double b[6];
    ren->ComputeVisiblePropBounds(b);
    if (!vtkMath::AreBoundsInitialized(b))
    {
      return NoData();
    }
    extents.min = { b[0], b[2], b[4] };
    extents.max = { b[1], b[3], b[5] };
    return 0;
  }

  long GetIsSelectionEmpty(navlib::bool_t& empty) const override
  {
    // F3D has no object selection, the whole model is navigated
    empty = true;
    return 0;
  }

  long GetSelectionExtents(navlib::box_t&) const override
  {
    return NoData();
  }

  long GetSelectionTransform(navlib::matrix_t&) const override
  {
    return NotSupported();
  }

  long SetSelectionTransform(const navlib::matrix_t&) override
  {
    return InvalidOperation();
  }

  //--------------------------------------------------------------------------
  // IPivot: the point the driver rotates around
  long GetPivotPosition(navlib::point_t& position) const override
  {
    position = { this->Pivot[0], this->Pivot[1], this->Pivot[2] };
    return 0;
  }

  long SetPivotPosition(const navlib::point_t& position) override
  {
    this->Pivot = { position.x, position.y, position.z };
    this->UpdatePivotMarker();
    return 0;
  }

  long IsUserPivot(navlib::bool_t& userPivot) const override
  {
    // Let the driver pick the pivot from what is under the cursor or view center
    userPivot = false;
    return 0;
  }

  long GetPivotVisible(navlib::bool_t& visible) const override
  {
    visible = this->PivotActor->GetVisibility() != 0;
    return 0;
  }

  long SetPivotVisible(bool visible) override
  {
    this->PivotActor->SetVisibility(visible);
    this->UpdatePivotMarker();
    this->Interactor.requestRender();
    return 0;
  }

  //--------------------------------------------------------------------------
  // IHit: the driver asks what geometry lies along a ray
  long SetHitLookFrom(const navlib::point_t& eye) override
  {
    this->HitFrom = { eye.x, eye.y, eye.z };
    return 0;
  }

  long SetHitDirection(const navlib::vector_t& direction) override
  {
    this->HitDirection = { direction.x, direction.y, direction.z };
    return 0;
  }

  long SetHitAperture(double aperture) override
  {
    this->HitAperture = aperture;
    return 0;
  }

  long SetHitSelectionOnly(bool) override
  {
    return 0;
  }

  long GetHitLookAt(navlib::point_t& position) const override
  {
    vtkRenderer* ren = this->Renderer();
    vtkCamera* cam = this->Camera();
    if (!ren || !cam)
    {
      return InvalidOperation();
    }

    // Find where the ray crosses the screen by projecting a point along it
    const double dist = std::max(cam->GetDistance(), 1e-6);
    double probe[4] = { this->HitFrom[0] + dist * this->HitDirection[0],
      this->HitFrom[1] + dist * this->HitDirection[1],
      this->HitFrom[2] + dist * this->HitDirection[2], 1.0 };
    ren->SetWorldPoint(probe);
    ren->WorldToDisplay();
    double display[3];
    ren->GetDisplayPoint(display);

    // Use the aperture as pick tolerance, relative to the view size
    const double viewHeight = this->VisibleHeightAtFocalPoint();
    const double tolerance =
      viewHeight > 0 ? std::clamp(this->HitAperture / viewHeight, 0.001, 0.05) : 0.005;
    this->Picker->SetTolerance(tolerance);

    if (this->Picker->Pick(display[0], display[1], 0.0, ren) == 0)
    {
      return NoData();
    }
    double hit[3];
    this->Picker->GetPickPosition(hit);
    position = { hit[0], hit[1], hit[2] };
    return 0;
  }

  //--------------------------------------------------------------------------
  // IState: motion frames
  long SetMotionFlag(bool) override
  {
    // Rendering is driven by SetTransaction, nothing to do here
    return 0;
  }

  long SetTransaction(long transaction) override
  {
    // 0 marks the end of a motion frame: draw it now so motion stays smooth
    if (transaction == 0 && this->CameraChanged)
    {
      this->CameraChanged = false;
      this->Interactor.UpdateRendererAfterInteraction();
      this->UpdatePivotMarker();
      this->Window.render();
    }
    return 0;
  }

  //--------------------------------------------------------------------------
  // IEvents
  long SetActiveCommand(std::string) override
  {
    // No F3D commands are exported to the driver yet
    return 0;
  }

  long SetSettingsChanged(long) override
  {
    return 0;
  }

private:
  // World directions of the navlib axes: right, up and out of the screen at the front view
  std::array<std::array<double, 3>, 3> SceneAxes() const
  {
    std::array<double, 3> up = { 0, 1, 0 };
    std::array<double, 3> right = { 1, 0, 0 };
    if (vtkF3DRenderer* ren = this->Window.GetRenderer())
    {
      ren->GetUpDirection(up.data());
      ren->GetRightDirection(right.data());
    }
    vtkMath::Normalize(up.data());
    // Make right exactly perpendicular to up
    const double d = vtkMath::Dot(right.data(), up.data());
    for (int i = 0; i < 3; i++)
    {
      right[i] -= d * up[i];
    }
    if (vtkMath::Normalize(right.data()) < 1e-6)
    {
      right = std::abs(up[0]) < 0.9 ? std::array<double, 3>{ 1, 0, 0 }
                                     : std::array<double, 3>{ 0, 1, 0 };
    }
    std::array<double, 3> out;
    vtkMath::Cross(right.data(), up.data(), out.data());
    return { right, up, out };
  }

  window_impl& Window;
  interactor_impl& Interactor;

  vtkNew<vtkCellPicker> Picker;
  vtkNew<vtkSphereSource> PivotSource;
  vtkNew<vtkPolyDataMapper> PivotMapper;
  vtkNew<vtkActor> PivotActor;

  std::array<double, 3> Pivot = { 0, 0, 0 };
  std::array<double, 3> HitFrom = { 0, 0, 0 };
  std::array<double, 3> HitDirection = { 0, 0, -1 };
  double HitAperture = 0.0;

  std::array<double, 6> LastBounds = { 0, 0, 0, 0, 0, 0 };
  bool LastPerspective = true;
  bool CameraChanged = false;
};

//----------------------------------------------------------------------------
spacemouse_impl::spacemouse_impl(window_impl& window, interactor_impl& interactor)
  : Internals(std::make_unique<internals>(window, interactor))
{
}

//----------------------------------------------------------------------------
spacemouse_impl::~spacemouse_impl() = default;

//----------------------------------------------------------------------------
bool spacemouse_impl::Enable()
{
  return this->Internals->Enable();
}

//----------------------------------------------------------------------------
void spacemouse_impl::Disable()
{
  this->Internals->Disable();
}

//----------------------------------------------------------------------------
void spacemouse_impl::Tick()
{
  this->Internals->Tick();
}
}
