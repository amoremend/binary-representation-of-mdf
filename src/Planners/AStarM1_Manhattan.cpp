#include "Planners/AStarM1_Manhattan.hpp"

namespace Planners{
    
AStarM1_Manhattan::AStarM1_Manhattan(bool _use_3d):AStar_Manhattan(_use_3d, "astarm1") {}
AStarM1_Manhattan::AStarM1_Manhattan(bool _use_3d, std::string _name = "astarm1" ):AStar_Manhattan(_use_3d, _name) {}

inline unsigned int AStarM1_Manhattan::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs){
    
    unsigned int cost = _current->G;

    if(_dirs == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_manhattan); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_manhattan : dd_3D_manhattan)); //This is more efficient
    }
 
    // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
    // IROS
    // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_current->cost + _suc->cost)/2) * dist_scale_factor_reduced_); //+grad_suc 
    // RA-L
    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
    cost += cost_term;
    _suc->C = cost_term;
    
    return cost;
}
}
