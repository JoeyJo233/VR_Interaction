
#include <gmGraphics/OsgRenderer.hh>
#include <gmTrack/Controller.hh>
#include <gmNetwork/SyncNode.hh>


using namespace gramods;

/**
 * My simple scene graph app.
 *
 * This demo applies the _pimpl_ technique to separate external
 * interface, used to call the app from the system, and internal
 * functionality that then can fully reside in `MyApp.cpp`.
 */
class MyApp {

public:
  /**
   * Called from main() to create the app.
   */
  MyApp(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
        std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
        std::shared_ptr<gmTrack::SinglePoseTracker> head);

  /**
   * For pimpl to work with std::unique_ptr the destructor must be
   * implemented at a point in the code where the struct Impl is
   * complete.
   */
  ~MyApp();

  /**
   * We set up our own internal scene graph. This is returned to
   * main() so that it can be added to the gmGraphics::Window.
   */
  std::shared_ptr<gmGraphics::Node> getSGRoot();

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};
