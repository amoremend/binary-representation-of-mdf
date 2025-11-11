#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP
/**
 * @file LineOfSight.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief Implementation of line of sight related algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "utils/utils.hpp"
#include "utils/world.hpp"
#include "utils/geometry_utils.hpp"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            /**
             * @brief 
             * 
             * @param _lnode 
             * @param _rnode 
             * @param _world 
             * @param _visited_nodes 
             * @return true 
             * @return false 
             */
            bool bresenham3D(const Vec3i _lnode, const Vec3i _rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes);
            // Whithout considering the occupied nodes
            bool bresenham3D_WON(const Vec3i _lnode, const Vec3i _rnode, CoordinateListPtr _visited_nodes);

            /**
             * @brief Implementation of bressenham 3D line of sight algorithm
             * 
             * @param _lnode First node
             * @param _rnode Second node
             * @param _world discrete world in which the nodes are stored. It is used
             * to check if the coordinates that the algorithm iterates are occupied/valids
             * @param _visited_nodes 
             * 
             * @return true If there exists line of sight between both nodes
             * @return false If there is no line of sight between both nodes
             */
            bool bresenham3D(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes = nullptr);
            // Whithout considering the occupied nodes
            bool bresenham3D_WON(const Node *_lnode, const Node *_rnode, CoordinateListPtr _visited_nodes = nullptr);

            // Bresenham considering Manhattan Distance
            bool bresenham3D_Man(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes);
            bool bresenham3D_Man(const Vec3i &_lnode, const Vec3i &_rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes);

            // Bresenham considering Manhattan Distance without considering the occupied nodes
            bool bresenham3D_Man_WON(const Node *_lnode, const Node *_rnode, CoordinateListPtr _visited_nodes);
            bool bresenham3D_Man_WON(const Vec3i &_lnode, const Vec3i &_rnode, CoordinateListPtr _visited_nodes);


            /**
             * @brief 
             * 
             * @param _lnode 
             * @param _rnode 
             * @param _world 
             * @param _threshold 
             * @return true 
             * @return false 
             */
            bool bresenham3DWithMaxThreshold(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, const unsigned int _threshold);
        
            /**
             * @brief 
             * 
             * @param _lnode 
             * @param _rnode 
             * @param _world 
             * @param _threshold 
             * @return int 
             */
            int nodesInLineBetweenTwoNodes(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, const unsigned int _threshold);
            // Number of nodes whithout considering the occupied nodes and max LoS
            int nodesInLineBetweenTwoNodes_WON(const Node *_lnode, const Node *_rnode);
        }
    }
}

#endif