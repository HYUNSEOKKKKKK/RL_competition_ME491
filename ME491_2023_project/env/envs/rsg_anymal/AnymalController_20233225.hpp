// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#pragma once

#include <set>
#include "../../BasicEigenTypes.hpp"
#include "raisim/World.hpp"

namespace raisim {

/// change the class name and file name ex) AnymalController_20233225 -> AnymalController_STUDENT_ID
class AnymalController_20233225 {

 public:
  inline bool create(raisim::World *world) {
    anymal_ = reinterpret_cast<raisim::ArticulatedSystem *>(world->getObject(name_));
    anymal_blue = reinterpret_cast<raisim::ArticulatedSystem *>(world->getObject(opponentName_));
    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_);
    gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_);
    vTarget_.setZero(gvDim_);
    pTarget12_.setZero(nJoints_);

    gc_blue.setZero(gcDim_);
    gv_blue.setZero(gvDim_);

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero();
    jointPgain.tail(nJoints_).setConstant(50.0);
    jointDgain.setZero();
    jointDgain.tail(nJoints_).setConstant(0.2);
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 2 + 18 + 19;
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.1);

    /// indices of links that should not make contact with ground
    footIndices_.insert(anymal_->getBodyIdx("LF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("LH_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RH_SHANK"));

    return true;
  }

  inline bool init(raisim::World *world) {
    return true;
  }

  inline bool advance(raisim::World *world, const Eigen::Ref<EigenVec> &action) {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;
    anymal_->setPdTarget(pTarget_, vTarget_);
//    raisim::Vec<3> force;
//    force = [0,0,-1]
//    anymal_->setExternalForce()
    return true;
  }

  inline bool advance_blue(raisim::World *world, const Eigen::Ref<EigenVec> &action) {
    /// action scaling
    pTarget12_.tail(12) = gc_init_.tail(12);
    pTarget_.tail(nJoints_) = pTarget12_.tail(12);
    anymal_->setPdTarget(pTarget_, vTarget_);
//    pTarget12_ = action.cast<double>();
//    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
//    pTarget12_ += actionMean_;
//    pTarget_.tail(nJoints_) = pTarget12_;
//    anymal_->setPdTarget(pTarget_, vTarget_);
    return true;
  }

  inline bool reset(raisim::World *world, double theta, double radius) {
    if (playerNum_ == 0) {
      gc_init_.head(3) << 1.5 * std::cos(theta), 1.5 * std::sin(theta), 0.5;
      gc_init_.segment(3, 4) << cos((theta - M_PI) / 2), 0, 0, sin((theta - M_PI) / 2);
    }
    else {
      gc_init_.head(3) << radius  * std::cos(theta + M_PI), radius * std::sin(theta + M_PI), 0.5;
      gc_init_.segment(3, 4) << cos(theta / 2), 0, 0, sin(theta / 2);
    }
    anymal_->setState(gc_init_, gv_init_);
    return true;
  }

  inline void updateObservation(raisim::World *world) {
      // my controller
    anymal_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    rot;
    quat[0] = gc_[3];
    quat[1] = gc_[4];
    quat[2] = gc_[5];
    quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    // blue controller
    anymal_blue->getState(gc_blue, gv_blue);
//    raisim::Vec<4> quat_blue;
//    raisim::Mat<3, 3> rot_blue;
//    quat_blue[0] = gc_blue[3];
//    quat_blue[1] = gc_blue[4];
//    quat_blue[2] = gc_blue[5];
//    quat_blue[3] = gc_blue[6];
//    raisim::quatToRotMat(quat_blue, rot_blue);


    obDouble_ <<
//        gc_[2], /// body pose
//        rot.e().row(2).transpose(), /// body orientation
//        gc_.tail(12), /// joint angles
//        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
//        gv_.tail(12), /// joint velocity
        gc_blue.head(2),
//        gv_blue,
        gc_,
        gv_;
  }

  inline void recordReward(Reward *rewards) {

      Eigen::Vector3d direc = rot.e().transpose()*(gc_blue.head(3)-gc_.head(3));
      Eigen::Vector3d direc_normalize = direc / direc.norm();

    rewards->record("directionVel", ((bodyLinearVel_.dot(direc_normalize))));
//    rewards->record("lower", (10.0 - gc_[2]));
//    rewards->record("speed", bodyLinearVel_.norm()*(-pow((gc_.head(2)-gc_blue.head(2)).norm(),2)));
//    rewards->record("getclose", exp(-pow((gc_.head(2)-gc_blue.head(2)).norm(),2)));
//    rewards->record("defense", gc_blue.head(2).norm());
//    rewards->record("push", gv_blue.head(2).norm());
  }

  inline const Eigen::VectorXd &getObservation() {
    return obDouble_;
  }

  void setName(const std::string &name) {
    name_ = name;
  }

  void setOpponentName(const std::string &name) {
    opponentName_ = name;
  }

  void setPlayerNum(const int &playerNum) {
    playerNum_ = playerNum;
  }

  inline bool isTerminalState_blue(raisim::World *world) {
    for (auto &contact: anymal_blue->getContacts()) {
        if (contact.getPairObjectIndex() == world->getObject("ground")->getIndexInWorld() &&
            contact.getlocalBodyIndex() == anymal_blue->getBodyIdx("base")) {
            return true;
        }
    }
    if (gc_blue.head(2).squaredNorm()>9){
        return true;
    }
    return false;
  }


  inline bool isTerminalState(raisim::World *world) {
    for (auto &contact: anymal_->getContacts()) {
      if (contact.getPairObjectIndex() == world->getObject("ground")->getIndexInWorld() &&
          contact.getlocalBodyIndex() == anymal_->getBodyIdx("base")) {
        return true;
      }
    }
    if (gc_.head(2).squaredNorm()>9){
        return true;
    }
    return false;
  }

  inline int getObDim() {
    return obDim_;
  }

  inline int getActionDim() {
    return actionDim_;
  }

    raisim::ArticulatedSystem *anymal_;
 private:
  std::string name_, opponentName_;
  // my controller

  int gcDim_, gvDim_, nJoints_, playerNum_ = 0;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
  // blue controller
  raisim::ArticulatedSystem  *anymal_blue;
  Eigen::VectorXd gc_blue, gv_blue;
  raisim::Mat<3, 3> rot;
  int obDim_ = 0, actionDim_ = 0;

};

}