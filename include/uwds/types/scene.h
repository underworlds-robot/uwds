#ifndef SCENE_HPP
#define SCENE_HPP

#include "nodes.h"
#include <queue>
#include <regex>
#include <pose_cov_ops/pose_cov_ops.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * This class represent the scene.
   */
  class Scene {
    public:
      /** @brief
       * Default constructor.
       */
      Scene() {
        nodes_ = boost::make_shared<Nodes>();
        root_id_ = nodes_->rootID();
      }

      /** @brief
       * Copy destructor.
       */
      Scene(const Scene& scene) = default;

      /** @brief
       * Move constructor.
       */
      Scene(Scene&& scene) = delete;

      /** @brief
       * Default destructor.
       */
      ~Scene() = default;

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       * @return The node id updated
       */
      string update(const NodePtr node)
      {
        nodes_->update(node);
        return node->id;
      }

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       * @return The node id updated
       */
      string update(const Node node)
      {
        nodes_->update(node);
        return node.id;
      }

      /** @brief
       * This method update a set of nodes (or create them if new)
       *
       * @param nodes The nodes to update
       * @return The node ids updated
       */
      vector<string> update(const vector<Node> nodes)
      {
        vector<string> node_ids;
        for (const auto& node : nodes)
        {
          node_ids.push_back(node.id);
          nodes_->update(node);
        }
        return node_ids;
      }

      /** @brief
       * This method update a set of nodes (or create them if new)
       *
       * @param nodes The nodes to update
       * @return The node ids updated
       */
      vector<string> update(const vector<NodePtr> nodes)
      {
        vector<string> node_ids;
        for (const auto& node : nodes)
        {
          node_ids.push_back(node->id);
          nodes_->update(node);
        }
        return node_ids;
      }

      /** @brief
       * This method remove a node
       *
       * @param id The node id to delete
       * @return The node id deleted
       */
      string remove(const string id)
      {
        nodes_->remove(id);
        return id;
      }

      /** @brief
       * This method remove a set of nodes
       *
       * @param ids The node ids to delete
       * @return The node ids deleted
       */
      vector<string> remove(const vector<string> ids)
      {
        nodes_->remove(ids);
        return ids;
      }

      /** @brief
       * This method set the root node ID
       *
       * @param root_id The root node ID
       */
      //string setRootID(const string& root_id) {root_id_=root_id;}

      /** @brief
       * This method return the root node
       *
       * @return The root node ID
       */
      string rootID() const {return root_id_;}

      /** @brief
       * This method return the nodes container.
       *
       * @return The nodes container
       */
      Nodes& nodes() {return * nodes_;}

      /** @brief
       * This method reset the scene.
       *
       * @return The nodes ids deleted
       */
      vector<string> reset(const string& new_root_id)
      {
        vector<string> node_ids;
        for (const auto& node : nodes())
          node_ids.push_back(node->id);
        root_id_ = new_root_id;
        nodes_ = boost::make_shared<Nodes>(root_id_);
        return node_ids;
      }

      /** @brief
       * Lock the scene.
       */
      void lock() {nodes_->lock();}

      /** @brief
       * Unlock the scene.
       */
      void unlock() {nodes_->unlock();}

      /** @brief
       * Get the nodes that have a name that math the given regex.
       *
       * @param regex The regex
       * @return The nodes that match
       */
      vector<Node> getNodesByName(const string& regex)
      {
        Node current_node;
        vector<Node> match;
        queue<Node> fifo;
        lock();
        fifo.push(nodes()[rootID()]);
        do {
          current_node = fifo.front();
          fifo.pop();
            if(std::regex_match(current_node.name, std::regex(regex)))
              match.push_back(current_node);
          for(const auto& child : current_node.children)
            fifo.push(nodes()[child]);
        } while(!fifo.empty());
        unlock();
        return match;
      }

      /** @brief
       * Get the given node parents.
       *
       * @param node_id The node ID
       * @return The parents of the node
       */
      vector<Node> getParents(const string& node_id)
      {
        Node current_node;
        vector<Node> parents;
        queue<Node> fifo;
        lock();
        if (nodes()[node_id].parent != "")
        {
          fifo.push(nodes()[nodes()[node_id].parent]);
          do {
            current_node = fifo.front();
            fifo.pop();
            parents.push_back(current_node);
            if(current_node.parent!="")
              fifo.push(nodes()[current_node.parent]);
          } while(!fifo.empty());
        }
        unlock();
        return parents;
      }

      geometry_msgs::Pose getWorldPose(const string& node_id)
      {
        Node current_node;
        geometry_msgs::Pose final_pose;
        queue<Node> fifo;
        lock();
        final_pose = nodes()[node_id].position.pose;
        if (nodes()[node_id].parent != "")
        {
          fifo.push(nodes()[nodes()[node_id].parent]);
          do {
            current_node = fifo.front();
            fifo.pop();
            pose_cov_ops::compose(current_node.position.pose, final_pose, final_pose);
            if(current_node.parent!="")
              fifo.push(nodes()[current_node.parent]);
          } while(!fifo.empty());
        }
        unlock();
        return final_pose;
      }

      geometry_msgs::PoseWithCovariance getWorldPoseWithCovariance(const string& node_id)
      {
        Node current_node;
        geometry_msgs::PoseWithCovariance final_pose;
        queue<Node> fifo;
        lock();
        final_pose = nodes()[node_id].position;
        if (nodes()[node_id].parent != "")
        {
          fifo.push(nodes()[nodes()[node_id].parent]);
          do {
            current_node = fifo.front();
            fifo.pop();
            pose_cov_ops::compose(current_node.position, final_pose, final_pose);
            if(current_node.parent!="")
              fifo.push(nodes()[current_node.parent]);
          } while(!fifo.empty());
        }
        unlock();
        return final_pose;
      }

      geometry_msgs::Pose lookUpPose(const string& source_id, const string& target_id)
      {
        Node current_node;
        geometry_msgs::Pose source_pose, target_pose, final_pose;
        queue<Node> fifo;
        source_pose = getWorldPose(source_id);
        target_pose = getWorldPose(target_id);
        pose_cov_ops::inverseCompose(target_pose, source_pose, final_pose);
        return final_pose;
      }

      geometry_msgs::PoseWithCovariance lookUpPoseWithCovariance(const string& source_id, const string& target_id)
      {
        Node current_node;
        geometry_msgs::PoseWithCovariance source_pose, target_pose, final_pose;
        queue<Node> fifo;
        source_pose = getWorldPoseWithCovariance(source_id);
        target_pose = getWorldPoseWithCovariance(target_id);
        pose_cov_ops::inverseCompose(target_pose, source_pose, final_pose);
        return final_pose;
      }

      bool has(const string id)
      {
        return nodes().has(id);
      }

      size_t size()
      {
        return nodes().size();
      }

    private:
      string root_id_;
      NodesPtr nodes_;
  };

  typedef boost::shared_ptr<uwds::Scene> ScenePtr;
  typedef boost::shared_ptr<uwds::Scene const> SceneConstPtr;
}

#endif
