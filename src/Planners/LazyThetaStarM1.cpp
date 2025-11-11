#include "Planners/LazyThetaStarM1.hpp"

namespace Planners
{
    LazyThetaStarM1::LazyThetaStarM1(bool _use_3d):LazyThetaStar(_use_3d, "lazythetastarm1") {}
    LazyThetaStarM1::LazyThetaStarM1(bool _use_3d, std::string _name = "lazythetastarm1" ):LazyThetaStar(_use_3d, _name) {}
    
    void LazyThetaStarM1::SetVertex(Node *_s_aux)
    {   
        if (!LineOfSight::bresenham3DWithMaxThreshold(_s_aux->parent, _s_aux, discrete_world_, max_line_of_sight_cells_)){
            line_of_sight_checks_++;
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr || successor2->occuppied ) continue;

                if ( successor2->isInClosedList ) 
                {
                    // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
                    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))/2) * dist_scale_factor_reduced_)));
                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term;
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->C      = cost_term;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
    }
    inline void LazyThetaStarM1::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        // // ROS_INFO("Using resolution: [%lf]", _s2_aux->cost);

        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        // std::cout << "distanceParent2: " << distanceParent2 << std::endl;
        // ROS_INFO("Compute COST");

        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes_WON(_s_aux->parent, _s2_aux);

        // Never is 1, so when distanceParent2_nodes == 0 it should be distanceParent2_nodes == 1 to compute cost_term
        if ( distanceParent2_nodes == 0 ){
            // std::cout << "distanceParent2_nodes: " << distanceParent2_nodes << std::endl;
            distanceParent2_nodes = 1;
        }

        // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost))/2) * dist_scale_factor_reduced_))) * distanceParent2_nodes; 
        auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost))/2) * distanceParent2_nodes * dist_scale_factor_reduced_))); 
        
        if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + distanceParent2 +  cost_term;
            _s2_aux->C      = cost_term;     
            _s2_aux->gplush = _s2_aux->G + _s2_aux->H;   
        }

        // ROS_INFO("Using resolution: [%lf]", _s2_aux->cost);

    }


    unsigned int LazyThetaStarM1::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        // SIMILAR TO COMPUTEG IN M2.
        unsigned int cost = 0;

        if(_dirs == 8){
            cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 

        cost += _current->G;
        cost += cost_term;
        _suc->C = cost_term;
        
        return cost;
    }
}