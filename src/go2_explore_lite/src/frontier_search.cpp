#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, costmap_2d::Costmap2D* traversal)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{
  traversal_ = traversal;
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  if (traversal_) return searchWithTraversal(position);
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

std::vector<Frontier> FrontierSearch::searchWithTraversal(geometry_msgs::Point position)
{
  std::vector<Frontier> result;
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> travel_lock(*traversal_->getMutex());
  if (costmap_->getSizeInCellsX()!=traversal_->getSizeInCellsX() ||
      costmap_->getSizeInCellsY()!=traversal_->getSizeInCellsY() ||
      std::abs(costmap_->getOriginX()-traversal_->getOriginX())>1e-5 ||
      std::abs(costmap_->getOriginY()-traversal_->getOriginY())>1e-5 ||
      std::abs(costmap_->getResolution()-traversal_->getResolution())>1e-6) return result;
  unsigned mx,my;
  if (!traversal_->worldToMap(position.x,position.y,mx,my)) return result;
  const unsigned pos=traversal_->getIndex(mx,my);
  const auto* travel=traversal_->getCharMap();
  map_=costmap_->getCharMap();
  size_x_=costmap_->getSizeInCellsX();size_y_=costmap_->getSizeInCellsY();
  const auto free=[&](unsigned i) {return travel[i]<costmap_2d::INSCRIBED_INFLATED_OBSTACLE;};
  unsigned start=pos;
  // Match the adapter's bounded nearest-free search if the center is inflated.
  if (!free(start)) {
    bool found=false;
    for (int r=1;r<=10 && !found;++r)
      for (int y=std::max(0,int(my)-r);y<std::min(int(size_y_),int(my)+r+1) && !found;++y)
        for (int x=std::max(0,int(mx)-r);x<std::min(int(size_x_),int(mx)+r+1);++x)
          if (free(y*size_x_+x)) {start=y*size_x_+x;found=true;break;}
    if (!found) return result;
  }
  std::vector<bool> reachable(size_x_*size_y_,false), flags(size_x_*size_y_,false);
  std::queue<unsigned> pending;pending.push(start);reachable[start]=true;
  while (!pending.empty()) {
    const unsigned i=pending.front();pending.pop();
    for (unsigned n:nhood4(i,*costmap_))
      if (!reachable[n] && free(n)) {reachable[n]=true;pending.push(n);}
  }
  eligible_.assign(reachable.size(),false);
  for (unsigned i=0;i<reachable.size();++i) if (reachable[i]) {
    if (map_[i]==NO_INFORMATION) eligible_[i]=true;
    for (unsigned n:nhood4(i,*costmap_)) if (map_[n]==NO_INFORMATION) eligible_[n]=true;
  }
  for (unsigned i=0;i<eligible_.size();++i) if (eligible_[i] && !flags[i]) {
    flags[i]=true;
    auto frontier=buildNewFrontier(i,pos,flags);
    if (frontier.size*costmap_->getResolution()>=min_frontier_size_) {
      frontier.cost=frontierCost(frontier);result.push_back(std::move(frontier));
    }
  }
  std::sort(result.begin(),result.end(),[](const Frontier& a,const Frontier& b){return a.cost<b.cost;});
  return result;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);
  // Upstream omitted the seed from points and the centroid sum but counted it
  // in size. Include it so the cluster geometry is independent of map origin.
  output.points.push_back(output.initial);
  output.centroid = output.initial;

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);
  output.min_distance = std::hypot(reference_x-output.initial.x,
                                  reference_y-output.initial.y);
  output.middle = output.initial;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  if (traversal_) return eligible_[idx] && !frontier_flag[idx];
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}
