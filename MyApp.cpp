
#include "MyApp.hh"

#include <gmCore/FileResolver.hh>
#include <gmCore/TimeTools.hh>
#include <gmCore/Updateable.hh>

#include <gmTrack/ButtonsMapper.hh>

#include <gmNetwork/RunSync.hh>
#include <gmNetwork/DataSync.hh>
#include <gmNetwork/SyncSData.hh>
#include <gmNetwork/SyncMData.hh>

#include <gmGraphics/Group.hh>
#include <gmGraphics/MatrixTransform.hh>
#include <gmGraphics/ObjRenderer.hh>

#include <string>
#include <gmGraphics/IntersectionVisitor.hh>

#include <filesystem>
#define _USE_MATH_DEFINES
#include <cmath>
#include <SDL_stdinc.h>
#include <map>

using namespace gramods;

typedef gmNetwork::SyncSData<Eigen::Vector3f> SyncSVec;
typedef gmNetwork::SyncSData<Eigen::Quaternionf> SyncSQuat;

/**
 * Declarations of the internal code of MyApp
 */
struct MyApp::Impl : public gmCore::Updateable {
  Impl(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
       std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
       std::shared_ptr<gmTrack::SinglePoseTracker> head);

  void setup_sync(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes);

  void setup_wand(std::vector<std::shared_ptr<gmTrack::Controller>> controllers);

  void add_obj(std::string str, double scale_factor, Eigen::Vector3f&);//function to be implemented

  /**
   * From gmCore::Updateable. Automatically called from main() via
   * gmCore::Updateable::updateAll();
   */
  void update(clock::time_point time, size_t frame) override;

  void update_data(clock::time_point time);

  void update_states(clock::time_point time);

  // Cluster synchronization handler
  std::shared_ptr<gmNetwork::SyncNode> sync_node;
  bool is_primary = true;

  // Wand (if this exists in the configuration)
  std::shared_ptr<gmTrack::Controller> wand;

  // Head (if this exists in the configuration)
  std::shared_ptr<gmTrack::SinglePoseTracker> head;

  /// ----- Containers for synchronized data -----

  // steady time
  std::shared_ptr<gmNetwork::SyncSFloat64> sync_time =
      std::make_shared<gmNetwork::SyncSFloat64>();

  // steady frame number
  std::shared_ptr<gmNetwork::SyncSUInt64> sync_frame_number =
      std::make_shared<gmNetwork::SyncSUInt64>(0);

  // wand analogs
  std::shared_ptr<gmNetwork::SyncMFloat32> sync_analogs =
      std::make_shared<gmNetwork::SyncMFloat32>();

  // wand main button
  std::shared_ptr<gmNetwork::SyncSBool> sync_main_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand second button
  std::shared_ptr<gmNetwork::SyncSBool> sync_second_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand menu button
  std::shared_ptr<gmNetwork::SyncSBool> sync_menu_button =
      std::make_shared<gmNetwork::SyncSBool>();

  // wand pose (position + orientation)
  std::shared_ptr<SyncSVec> sync_wand_position = std::make_shared<SyncSVec>();
  std::shared_ptr<SyncSQuat> sync_wand_orientation =
      std::make_shared<SyncSQuat>(Eigen::Quaternionf::Identity());

  // head pose (position + orientation)
  std::shared_ptr<SyncSVec> sync_head_position = std::make_shared<SyncSVec>();
  std::shared_ptr<SyncSQuat> sync_head_orientation =
      std::make_shared<SyncSQuat>(Eigen::Quaternionf::Identity());

  /// ----- Scene Graph Stuff -----

  std::shared_ptr<gmGraphics::Group> scenegraph_root;
  std::shared_ptr<gmGraphics::MatrixTransform> wand_transform_node;
  std::shared_ptr<gmGraphics::ObjRenderer> wand_object_node;
  
  
  std::shared_ptr<gmGraphics::MatrixTransform> cube_transform_node;
  std::shared_ptr<gmGraphics::ObjRenderer> cube_object_node;

  //
  bool is_navigating = false;
  float navigation_speed = 0.5f;
  float max_speed = 5.0f;

  float previous_wand_distance = 0.0f;

  std::shared_ptr<gmGraphics::MatrixTransform> all_obj_transform_node = std::make_shared<gmGraphics::MatrixTransform>();
  Eigen::Vector3f acc_position = Eigen::Vector3f::Zero();

  std::shared_ptr<gmGraphics::ObjRenderer> select_obj = nullptr;
  std::shared_ptr<gmGraphics::ObjRenderer> previous_obj = nullptr;

  float initial_wand_distance = 0.0f;

  //task8

  bool use_teleportation = false;
  float teleportation_distance = 5.0f;

  bool was_navigating = false;

  bool is_grabbing = false;
  std::shared_ptr<gmGraphics::MatrixTransform> grabbed_transform = nullptr;
  Eigen::Affine3f grab_offset;
  std::map < std::shared_ptr<gmGraphics::ObjRenderer>, std::shared_ptr<gmGraphics::MatrixTransform>> obj_to_transform_map;

  //task10 HOMER
  bool use_homer = false;
  bool is_homer_manipulating = false;
  std::shared_ptr<gmGraphics::MatrixTransform> homer_transform = nullptr;
  float homer_distance = 1.0f;
  Eigen::Quaternionf homer_orientation = Eigen::Quaternionf::Identity();
  //

  void initSG();
};


/// ----- External interface implementation -----

MyApp::MyApp(std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
             std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
             std::shared_ptr<gmTrack::SinglePoseTracker> head)
  : _impl(std::make_unique<Impl>(sync_nodes, controllers, head)) {}

MyApp::~MyApp() {}


std::shared_ptr<gmGraphics::Node> MyApp::getSGRoot() {
  return _impl->scenegraph_root;
}


/// ----- Internal implementation -----


MyApp::Impl::Impl(
    std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes,
    std::vector<std::shared_ptr<gmTrack::Controller>> controllers,
    std::shared_ptr<gmTrack::SinglePoseTracker> head)
  : head(head) {

  setup_sync(sync_nodes);
  setup_wand(controllers);
  
  initSG();
  
  all_obj_transform_node->initialize();
}

void MyApp::Impl::setup_sync(
    std::vector<std::shared_ptr<gmNetwork::SyncNode>> sync_nodes) {

  if (!sync_nodes.empty())
    // There should be only one SyncNode, but any excess will be
    // ignored anyways.
    sync_node = sync_nodes[0];
  else {
    // The config did not provide a SyncNode, so create one! We need
    // this so that we can use the sync_* variables even on a single
    // node without peers.
    sync_node = std::make_shared<gmNetwork::SyncNode>();
    sync_node->setLocalPeerIdx(0);
    sync_node->addPeer(""); //< Anything here since our own peer will
                            //  be ignored
    sync_node->initialize();
  }

  if (sync_node->getLocalPeerIdx() != 0) is_primary = false;

  // It is good practice to limit the use of raw pointers to the scope
  // in which you got it, however gmNetwork will keep the raw pointers
  // valid until sync_node is destroyed.
  gmNetwork::DataSync *data_sync =
      sync_node->getProtocol<gmNetwork::DataSync>();

  // Do not forget to add all containers to the synchronization queue
  data_sync->addData(sync_time);
  data_sync->addData(sync_frame_number);
  data_sync->addData(sync_analogs);
  data_sync->addData(sync_main_button);
  data_sync->addData(sync_second_button);
  data_sync->addData(sync_menu_button);
  data_sync->addData(sync_head_position);
  data_sync->addData(sync_head_orientation);
  data_sync->addData(sync_wand_position);
  data_sync->addData(sync_wand_orientation);
}

void MyApp::Impl::setup_wand(
    std::vector<std::shared_ptr<gmTrack::Controller>> controllers) {

  if (controllers.empty())
    // With no wand we are done setting up wands
    return;

  // Only the primary node should handle wand (the replica get the
  // data via network synchronization) but we keep a reference anyways
  // just so that we know to expect wand data.
  // if (!is_primary) return;

  // We could save away more than one wand, but one is enough for now
  wand = controllers[0];
}

void MyApp::Impl::update(gmCore::Updateable::clock::time_point time, size_t frame) {

  // Wait until we are connected to all peers before starting to
  // update data, animate and stuff
  if (!sync_node->isConnected())
    return;

  gmNetwork::DataSync *data_sync =
      sync_node->getProtocol<gmNetwork::DataSync>();
  gmNetwork::RunSync *run_sync =
      sync_node->getProtocol<gmNetwork::RunSync>();

  update_data(time);   // Let the primary update internal data

  run_sync->wait();    // Wait for primary to have sent its values
  data_sync->update(); // Swap old values for newly synchronized

  update_states(time); // Use the data to update scenegraph states
}

void MyApp::Impl::update_data(gmCore::Updateable::clock::time_point time) {

  if (!is_primary)
    // Only primary update internal states, the rest wait for incoming
    // data via the DataSync instance.
    return;
  if (*sync_main_button) { is_navigating = true;
  }
  else { is_navigating = false; }

  // Setting data to a SyncData instance (that has been added to a
  // DataSync instance) will send this value to all other nodes and
  // end up in the corresponding instance's back buffer.

  *sync_time = gmCore::TimeTools::timePointToSeconds(time);
  *sync_frame_number = *sync_frame_number + 1;

  if (head) {
    gmTrack::PoseTracker::PoseSample pose;
    if (head->getPose(pose)) {
      *sync_head_position = pose.position;
      *sync_head_orientation = pose.orientation;
    }
  }



  if (!wand)
    // Only wand stuff below this point, so terminate early if we do
    // not have a wand
    return;

  gmTrack::AnalogsTracker::AnalogsSample analogs;
  if (wand->getAnalogs(analogs))
    *sync_analogs = analogs.analogs;

  gmTrack::ButtonsTracker::ButtonsSample buttons;
  if (wand->getButtons(buttons)) {

    typedef gmTrack::ButtonsMapper::ButtonIdx ButtonIdx;

    if (buttons.buttons.count(ButtonIdx::MAIN))
      *sync_main_button = buttons.buttons[ButtonIdx::MAIN];
    else
      *sync_main_button = false;

    if (buttons.buttons.count(ButtonIdx::SECONDARY))
      *sync_second_button = buttons.buttons[ButtonIdx::SECONDARY];
    else
      *sync_second_button = false;

    if (buttons.buttons.count(ButtonIdx::MENU))
      *sync_menu_button = buttons.buttons[ButtonIdx::MENU];
    else
      *sync_menu_button = false;
  }

  gmTrack::PoseTracker::PoseSample pose;
  if (wand->getPose(pose)) {
    *sync_wand_position = pose.position;
    *sync_wand_orientation = pose.orientation;
  }
}

void MyApp::Impl::update_states(gmCore::Updateable::clock::time_point time) {
    static auto last_time = time;
    std::chrono::duration<float> delta_time = time - last_time;
    last_time = time;
  if (!wand_transform_node)
    return;




  Eigen::Vector3f eP = *sync_wand_position;
  Eigen::Quaternionf eQ = *sync_wand_orientation;

  Eigen::Vector3f wand_forward = eQ * Eigen::Vector3f(0,0,-1);
  Eigen::Vector3f ray_origin = eP;

  Eigen::Vector3f  head_position = *sync_head_position;
  float wand_distance = (eP - head_position).norm();
  //Eigen::Vector3f ray_direction = wand_forward.normalized();

  // task 10 ------------------------------------------ Toggle teleport & Homer
  static bool last_second_botton = false;
  static gmCore::Updateable::clock::time_point last_second_press_time = time;

  



  //toggle tele mode with menu button
  if (*sync_second_button && !last_second_botton) {
      auto time_since_last_press = std::chrono::duration<float>(time - last_second_press_time).count();
      if (time_since_last_press < 1.f) {
          use_homer = !use_homer;
          std::cout << " ^^ use homer mode HHHHHHHHHHHHHHHHHHH" << use_homer << std::endl;
      }
      else {
      use_teleportation = !use_teleportation;
          std::cout << " ^^ telemode" << use_teleportation << std::endl;
      }
      last_second_press_time = time;
  }
  
  last_second_botton = *sync_second_button;
  



  Eigen::Vector3f ray_direction = wand_forward.normalized();
  if (!is_navigating ) {
      previous_wand_distance = wand_distance;
      initial_wand_distance = wand_distance;

  }


  if (is_navigating) {

      if (use_teleportation && !was_navigating) {
          Eigen::Vector3f teleport_position = eP + wand_forward.normalized() * teleportation_distance;
          acc_position -= (teleport_position - head_position);
          //acc_position = -teleport_position;
          

          Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
          matrix.translate(-teleport_position);
          

          matrix.rotate(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitY()));

          all_obj_transform_node->setMatrix(matrix.matrix());
          //is_navigating = false;
          navigation_speed = 0;
          

          std::cout << "Teleport to" << teleport_position.transpose() << std::endl;
      }
      else {

          float distance_delta = wand_distance - initial_wand_distance;
          navigation_speed = distance_delta * 5.0f;
          navigation_speed = std::max(-max_speed, std::min(max_speed, distance_delta * 5.0f));

          //Eigen::Vector3f navigation_direction = (distance_delta > 0) ? wand_forward : -wand_forward;
          Eigen::Vector3f navigation_direction = wand_forward.normalized();


          //adding navigation direction 
          /*Eigen::Vector3f head_position = *sync_head_position;
          Eigen::Vector3f wand_position = *sync_wand_position;

          Eigen::Vector3f direction = wand_position - head_position;

          direction.normalize();

          Eigen::Vector3f translation = navigation_speed * direction * delta_time.count();
          */
          Eigen::Vector3f translation = navigation_speed * navigation_direction * delta_time.count();
          acc_position += translation;



          Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
          matrix.translate(acc_position);
          all_obj_transform_node->setMatrix(matrix.matrix());
          //Eigen::Affine3f current_transform = scenegraph_root->getNodes();
      }

  }

  //Task9: grab
  
  static bool last_menu_button = false;
  if (*sync_menu_button) {
      //std::cout << "Pressing sync_menu_button :) " << std::endl;
  }


  if (*sync_menu_button && !last_menu_button && select_obj && !is_grabbing && !is_homer_manipulating) { 
    //Task 10
    if (use_homer) {
      // Start HOMER manipulation
      is_homer_manipulating = true;
      homer_transform = obj_to_transform_map[select_obj];
      if (homer_transform) {
          Eigen::Affine3f parent_transform(all_obj_transform_node->getMatrix());
          Eigen::Affine3f obj_matrix = parent_transform * Eigen::Affine3f(homer_transform->getMatrix());
          Eigen::Vector3f obj_pos = obj_matrix.translation();
          homer_distance = (obj_pos - head_position).norm() / (eP - head_position).norm();
          homer_distance = std::max(0.1f, homer_distance); // Prevent zero/negative distance
          homer_orientation = Eigen::Quaternionf(obj_matrix.rotation());
          std::cout << "Starting HOMER manipulation: distance = " << homer_distance << std::endl;
      } else {
          std::cout << "Error: No transform found for selected object" << std::endl;
          is_homer_manipulating = false;
      }
  }
  else{  
    //Task 9
    // Start grabbing  
      is_grabbing = true;  
      grabbed_transform = obj_to_transform_map[select_obj];
      if (grabbed_transform) {
          Eigen::Affine3f wand_matrix = Eigen::Translation3f(eP) * eQ;

          Eigen::Affine3f parent_transform(all_obj_transform_node->getMatrix());
          Eigen::Affine3f obj_matrix = parent_transform* Eigen::Affine3f(grabbed_transform->getMatrix());


          grab_offset = wand_matrix.inverse() * obj_matrix; // Offset from wand to object  
          std::cout << "Grabbing object: " << select_obj.get() << std::endl; // Debug 
      }
      else {
          std::cout << "Error Grabbing" << std::endl;
          is_grabbing = false;
      }
    }
  }
  if (!*sync_menu_button && is_grabbing) { 
      // Stop grabbing  
      is_grabbing = false;  
      grabbed_transform = nullptr;  
      std::cout << "Released object" << std::endl; // Debug  
    }

  last_menu_button = *sync_menu_button; 

  // Update grabbed object�s position and orientation 
  if (is_grabbing && grabbed_transform) {  
      Eigen::Affine3f wand_matrix = Eigen::Translation3f(eP) * eQ;  
      Eigen::Affine3f obj_world_matrix = wand_matrix * grab_offset;  
      
      Eigen::Affine3f parent_inverse = Eigen::Affine3f(all_obj_transform_node->getMatrix()).inverse();
      Eigen::Affine3f local_matrix = parent_inverse * obj_world_matrix;
      
      
      grabbed_transform->setMatrix(local_matrix.matrix());


  }
  // homer release the obj
  if (!*sync_menu_button && is_homer_manipulating) { 
    is_homer_manipulating = false;
    homer_transform = nullptr;
    std::cout << "let go of obj Homer " << std::endl;
  }


  // Update HOMER-manipulated object (Task 10)
  if (is_homer_manipulating && homer_transform) {
    Eigen::Vector3f new_position = head_position + (eP - head_position ) * homer_distance;

    Eigen::Affine3f world_matrix = Eigen::Affine3f::Identity();
    world_matrix.translate(new_position);
    world_matrix.rotate(eQ);
    world_matrix.scale(0.08);
    Eigen::Affine3f parent_inverse = Eigen::Affine3f(all_obj_transform_node->getMatrix()).inverse();
    Eigen::Affine3f local_matrix = parent_inverse * world_matrix;
    homer_transform->setMatrix(local_matrix.matrix());
}

  


  std::shared_ptr<gmGraphics::ObjRenderer> closest_obj = nullptr;
  float closest_distance = std::numeric_limits<float>::max();
  if (!is_homer_manipulating) {
  gmGraphics::IntersectionVisitor intersection_vistor(gmGraphics::IntersectionLine::forwardRay(ray_origin, ray_direction));
  
  
  all_obj_transform_node->accept(&intersection_vistor);

  for (const auto& intersection : intersection_vistor.intersections) {
      //if (intersection.node_path.back() == wand_object_node.get()) continue;
      Eigen::Vector3f hit_position = intersection.position;
      float distance = (hit_position - eP).norm();

      gmGraphics::ObjRenderer* node = dynamic_cast<gmGraphics::ObjRenderer*>(intersection.node_path.back());
      if (node && distance < closest_distance) {
          closest_distance = distance;

          closest_obj = std::shared_ptr<gmGraphics::ObjRenderer>(node, [](gmGraphics::ObjRenderer*) {});
      }
  
  }


  if (closest_obj != select_obj) {
  /*      if (previous_obj) {
          auto materials = previous_obj->getMaterials();
          if (!materials.empty()) {
              materials.front().color_diffuse = Eigen::Vector3f(0.4, 0.4, 0.4);
              previous_obj->setMaterials(materials);
          }
      
      }*/

      if (select_obj) {
          auto materials = select_obj->getMaterials();
          if (!materials.empty()) {
              materials.front().color_diffuse = Eigen::Vector3f(0.4, 0.4, 0.4);
              select_obj->setMaterials(materials);
          }

      }




      if (closest_obj) {
          std::cout << closest_obj << std::endl;
          auto materials = closest_obj->getMaterials();
          if (!materials.empty()) {
              materials.front().color_diffuse = Eigen::Vector3f(1.0, 0.8, 0.0);
              closest_obj->setMaterials(materials);
          }
      }

     // previous_obj = select_obj;
      select_obj = closest_obj;
  }
}


  Eigen::Affine3f M = Eigen::Translation3f(eP) * eQ;
  wand_transform_node->setMatrix(M.matrix());

  double R = 0.4, G = 0.4, B = 0.4;
  if (*sync_main_button) R = 0.8;
  if (*sync_second_button) G = 0.8;
  if (*sync_menu_button) B = 0.8;

  std::vector<gmGraphics::ObjRenderer::Material> materials =
      wand_object_node->getMaterials();
  if (!materials.empty()) {
    materials.front().color_diffuse = Eigen::Vector3f(R, G, B);
    wand_object_node->setMaterials(materials);
  }



  was_navigating = is_navigating;

}




void MyApp::Impl::add_obj(std::string str,double scale_factor, Eigen::Vector3f& translationVector) {

    cube_transform_node = std::make_shared<gmGraphics::MatrixTransform>();
    cube_transform_node->initialize();
    Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
    // Eigen::Vector3f translationVector(0.2f, 0.1f, 0.0f);//0.2f, 0.1f, 0.0f for obj.obj
    matrix.translate(translationVector);
    matrix.scale(scale_factor); //0.008f for obj.obj
    cube_transform_node->setMatrix(matrix.matrix());
    all_obj_transform_node->addNode(cube_transform_node);


    cube_object_node = std::make_shared<gmGraphics::ObjRenderer>();
    cube_object_node->setFile(str);
    cube_object_node->initialize();
    cube_transform_node->addNode(cube_object_node);

    obj_to_transform_map[cube_object_node] = cube_transform_node;

}



void MyApp::Impl::initSG() {

  scenegraph_root = std::make_shared<gmGraphics::Group>();
  scenegraph_root->initialize();

  if (wand) {
    // We just have to assume that if a replica should render a wand,
    // because wand data come from the primary, then also the replica
    // will have a wand defined in their config, even if its data are
    // not used. How can we otherwise at setup know if we will render
    // a wand or not?

    wand_transform_node = std::make_shared<gmGraphics::MatrixTransform>();
    wand_transform_node->initialize();
    scenegraph_root->addNode(wand_transform_node);

    wand_object_node = std::make_shared<gmGraphics::ObjRenderer>();
    wand_object_node->setFile("urn:gramods:gmGraphics/resources/wand.obj");
    wand_object_node->initialize();
    wand_transform_node->addNode(wand_object_node);

    auto & trans = Eigen::Vector3f(0.2f, 0.1f, 0.0f);
    scenegraph_root->addNode(all_obj_transform_node);
    //add_obj("obj_models/rp_mei_30k.obj",0.008f, Eigen::Vector3f(0.2f, 0.1f, 0.0f));
    add_obj("obj_models/12140_Skull_v3_L2.obj", 0.008f, Eigen::Vector3f(-1*0.5f, 0.4f, 0.0f));
    //add_obj("obj_models/rp_nathan.obj", 0.008f, Eigen::Vector3f(0.0f, -1*0.5f, 0.0f));
    add_obj("obj_models/rp_sophia.obj", 0.008f, Eigen::Vector3f(0.4f, -1*0.1f, 0.4f));
    add_obj("cube.obj", 0.08f, Eigen::Vector3f(-1*0.4f, -1 * 0.3f, -1*0.4f));

  }
}









/*
*             // Create an ObjRenderer to render the .obj file
            auto objRenderer = std::make_shared<gmGraphics::ObjRenderer>();
            objRenderer->setFile(objFilePath); // Load the model
            objRenderer->initialize();
         

            // Create a MatrixTransform for positioning, scaling, and rotation
            auto transformNode = std::make_shared<gmGraphics::MatrixTransform>();

            Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
            Eigen::Vector3f translationVector(1.0f, 3.0f, 3.0f);
            matrix.translate(translationVector);
            transformNode->setMatrix(matrix.matrix());
            matrix.scale(3.0f);
*/