#ifndef ASTARM1_Manhattan_HPP
#define ASTARM1_Manhattan_HPP
/**
 * @file AStarM1_Manhattan.hpp
  * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This algorithm is a variation of the A*. The only
 * difference is that it reimplements the computeG method adding the
 * following cost term to the resturned result:
 * 
 *  auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc * dist_scale_factor_reduced_);
 * 
 * @version 0.1
 * @date 2025-10-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <Planners/AStar_Manhattan.hpp>

namespace Planners{

    /**
     * @brief 
     * 
     */
    class AStarM1_Manhattan : public AStar_Manhattan
    {
        
    public:
        /**
         * @brief Construct a new AStarM1 object
         * @param _use_3d This parameter allows the user to choose between 
         * planning on a plane (8 directions possibles) 
         * or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        AStarM1_Manhattan(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Cost Aware A Star M1 object
         * 
         * @param _use_3d 
         */
        AStarM1_Manhattan(bool _use_3d);

    protected:
        
        /**
         * @brief Overrided ComputeG function. 
         * 
         * @param _current Pointer to the current node
         * @param _suc Pointer to the successor node
         * @param _n_i The index of the direction in the directions vector. 
         * Depending on this index, the distance wi
         * @param _dirs Number of directions used (to distinguish between 2D and 3D)
         * @return unsigned int The G Value calculated by the function
         */
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    };

}

#endif 
