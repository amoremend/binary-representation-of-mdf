#include "Planners/LazyThetaStarM2.hpp"

namespace Planners
{
    LazyThetaStarM2::LazyThetaStarM2(bool _use_3d):ThetaStarM2(_use_3d, "lazythetastarm2") {}
    LazyThetaStarM2::LazyThetaStarM2(bool _use_3d, std::string _name = "lazythetastarm2" ):ThetaStarM2(_use_3d, _name) {}
    
    void LazyThetaStarM2::SetVertex(Node *_s_aux)
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

    inline void LazyThetaStarM2::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        // Like Original Lazy Theta Star
        auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        
        // checked_nodes is never 1 in LineOfSight::bresenham3D_WON(_s_aux->parent, _s2_aux, checked_nodes), but there is not a problem because in that case ComputeEdgeCost computes the cost
        // as a neighbour node
        LineOfSight::bresenham3D_WON(_s_aux->parent, _s2_aux, checked_nodes);
        // std::cout << "checked_nodes: " << checked_nodes->size() << std::endl;
        auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);
        // std::cout << "edge2" << edge2 << std::endl;
        if ( ( _s_aux->parent->G + dist2 + edge2 ) < _s2_aux->G ) 
        {   
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + dist2 + edge2;
            _s2_aux->C      = edge2;     
            _s2_aux->gplush = _s2_aux->G + _s2_aux->H;   
        }
         
        checked_nodes->clear();
    }

    inline unsigned int LazyThetaStarM2::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
        // SIMILAR TO COMPUTEG IN M1.
        unsigned int cost = 0;

        if(_dirs == 8){
            cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        auto edge_neighbour = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
    
        cost += _current->G;
        cost += edge_neighbour;
        _suc->C = edge_neighbour;
        
        return cost;
    }

    PathData LazyThetaStarM2::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        utils::Clock main_timer;
        main_timer.tic();

        MagicalMultiSet openSet;

        Node *current = nullptr;
        line_of_sight_checks_ = 0;

        bool solved{false};

        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);

        node_by_cost& indexByCost              = openSet.get<IndexByCost>();
        node_by_position& indexByWorldPosition = openSet.get<IndexByWorldPosition>();

        indexByCost.insert(discrete_world_.getNodePtr(_source));
        while (!indexByCost.empty())
        {
            auto it = indexByCost.begin();
            current = *it;
            indexByCost.erase(indexByCost.begin());
        
            if (current->coordinates == _target)
            {
                solved = true;
                break;
            }
            closedSet_.push_back(current);

            current->isInOpenList = false;
            current->isInClosedList = true;

            SetVertex(current);
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet_);
#endif
            exploreNeighbours(current, _target, indexByWorldPosition);
        }
        main_timer.toc();
        PathData result_data = createResultDataObject(current, main_timer, closedSet_.size(), 
                                                  solved, _source, line_of_sight_checks_);
   
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif
        closedSet_.clear();
        delete discrete_world_.getNodePtr(_source)->parent;

        discrete_world_.resetWorld();
        return result_data;
    }
}
