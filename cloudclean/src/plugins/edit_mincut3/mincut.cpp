#include "mincut.h"

#include <pcl/segmentation/boost.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <stdlib.h>
#include <cmath>
#include <pcl/features/fpfh.h>
#include <QDebug>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MinCut::MinCut () :
  inverse_sigma_ (16.0),
  binary_potentials_are_valid_ (false),
  epsilon_ (0.0001),
  radius_ (16.0),
  unary_potentials_are_valid_ (false),
  source_weight_ (0.8),
  search_ (),
  number_of_neighbours_ (14),
  graph_is_valid_ (false),
  foreground_points_ (0),
  background_points_ (0),
  clusters_ (0),
  graph_ (),
  capacity_ (),
  reverse_edges_ (),
  vertices_ (0),
  edge_marker_ (0),
  source_ (),/////////////////////////////////
  sink_ (),///////////////////////////////////
  horisonal_radius_(false),
  max_flow_ (0.0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MinCut::~MinCut ()
{
  if (search_ != 0)
    search_.reset ();
  if (graph_ != 0)
    graph_.reset ();
  if (capacity_ != 0)
    capacity_.reset ();
  if (reverse_edges_ != 0)
    reverse_edges_.reset ();

  foreground_points_.clear ();
  background_points_.clear ();
  clusters_.clear ();
  vertices_.clear ();
  edge_marker_.clear ();
  input_.reset ();
  indices_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setInputCloud (PointCloud::Ptr &cloud)
{
  input_ = cloud;
  graph_is_valid_ = false;
  unary_potentials_are_valid_ = false;
  binary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getSigma () const
{
  return (pow (1.0 / inverse_sigma_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSigma (double sigma)
{
  if (sigma > epsilon_)
  {
    inverse_sigma_ = 1.0 / (sigma * sigma);
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getRadius () const
{
  return (pow (radius_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setRadius (double radius)
{
  if (radius > epsilon_)
  {
    radius_ = radius * radius;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getSourceWeight () const
{
  return (source_weight_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSourceWeight (double weight)
{
  if (weight > epsilon_)
  {
    source_weight_ = weight;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MinCut::KdTreePtr
MinCut::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 unsigned int
MinCut::getNumberOfNeighbours () const
{
  return (number_of_neighbours_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setNumberOfNeighbours (unsigned int neighbour_number)
{
  if (number_of_neighbours_ != neighbour_number && neighbour_number != 0)
  {
    number_of_neighbours_ = neighbour_number;
    graph_is_valid_ = false;
    unary_potentials_are_valid_ = false;
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >
MinCut::getForegroundPoints () const
{
  return (foreground_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setForegroundPoints ( pcl::PointCloud<pcl::PointXYZI>::Ptr foreground_points)
{
  foreground_points_.clear ();
  foreground_points_.reserve (foreground_points->points.size ());
  for (size_t i_point = 0; i_point < foreground_points->points.size (); i_point++)
    foreground_points_.push_back (foreground_points->points[i_point]);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >
MinCut::getBackgroundPoints () const
{
  return (background_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setBackgroundPoints ( pcl::PointCloud<pcl::PointXYZI>::Ptr background_points)
{
  background_points_.clear ();
  background_points_.reserve (background_points->points.size ());
  for (size_t i_point = 0; i_point < background_points->points.size (); i_point++)
    background_points_.push_back (background_points->points[i_point]);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  // copy was here

  clusters_.clear ();
  bool success = true;

  if ( !graph_is_valid_ )
  {
    success = buildGraph ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    graph_is_valid_ = true;
    unary_potentials_are_valid_ = true;
    binary_potentials_are_valid_ = true;
  }

  if ( !unary_potentials_are_valid_ )
  {
    success = recalculateUnaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    unary_potentials_are_valid_ = true;
  }

  if ( !binary_potentials_are_valid_ )
  {
    success = recalculateBinaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    binary_potentials_are_valid_ = true;
  }

  // All checks are done here

  //IndexMap index_map = boost::get (boost::vertex_index, *graph_);
  ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

  max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

  assembleLabels (residual_capacity);

  // Copy current clusters to input ref?? WHY?
  if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
  {
    clusters.reserve (clusters_.size ());
    std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));
    deinitCompute ();
    return;
  }

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getMaxFlow () const
{
  return (max_flow_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  boost::shared_ptr< MinCut::mGraph>
MinCut::getGraph () const
{
  return (graph_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::buildGraph ()
{

     // Calculate fpfhs
     pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
     fpfh.setInputCloud (input_);
     fpfh.setInputNormals (normals_);
     fpfh.setIndices(indices_);
     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
     fpfh.setSearchMethod (tree);
     fpfh.setKSearch(14);
     fpfhs_ = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
     fpfh.compute (*fpfhs_);

  int number_of_points = static_cast<int> (input_->points.size ());
  int number_of_indices = static_cast<int> (indices_->size ());

  if (input_->points.size () == 0 || number_of_points == 0 || foreground_points_.empty () == true )
    return (false);

  if (search_ == 0)
    search_ = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> > (new pcl::search::KdTree<pcl::PointXYZI>);

  graph_.reset ();
  graph_ = boost::shared_ptr< mGraph > (new mGraph ());

  // Checks done, empty graph

  capacity_.reset ();
  capacity_ = boost::shared_ptr<CapacityMap> (new CapacityMap ());
  *capacity_ = boost::get (boost::edge_capacity, *graph_);

  // Capacity map configured

  reverse_edges_.reset ();
  reverse_edges_ = boost::shared_ptr<ReverseEdgeMap> (new ReverseEdgeMap ());
  *reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

  // Reverse edge map configured

  VertexDescriptor vertex_descriptor(0);
  vertices_.clear ();
  vertices_.resize (number_of_points + 2, vertex_descriptor);

  // Added 2 new vertices with 0 descriptors

  std::set<int> out_edges_marker;
  edge_marker_.clear ();
  edge_marker_.resize (number_of_points + 2, out_edges_marker);

  // Added two new out edge markers? avoid duplicate adds

  // Add vertices who have default indices (I assume)
  for (int i_point = 0; i_point < number_of_points + 2; i_point++)
    vertices_[i_point] = boost::add_vertex (*graph_);

  // Last two indices are the source and sink
  source_ = vertices_[number_of_points];
  sink_ = vertices_[number_of_points + 1];

  // Link up every point to the source and sink, with weights
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    double source_weight = 0.0; // Foreground penalty
    double sink_weight = 0.0; // Background penalty
    calculateUnaryPotential (point_index, source_weight, sink_weight);
    addEdge (static_cast<int> (source_), point_index, source_weight); // Connect to source
    addEdge (point_index, static_cast<int> (sink_), sink_weight); // Connect to sink
  }

  // Set the neighbours for every point
  // These are binary edges with weights
  std::vector<int> neighbours;
  std::vector<float> distances;
  search_->setInputCloud (input_, indices_);
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point]; // Get index in cloud
    search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
    for (size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++) // WHY skip the first neighbour?
    {
      double weight = calculateBinaryPotential (point_index, neighbours[i_nghbr]); // pass in cloud indices??
      addEdge (point_index, neighbours[i_nghbr], weight);
      addEdge (neighbours[i_nghbr], point_index, weight);
    }
    neighbours.clear ();
    distances.clear ();
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
  // Given an abritrary point in the cloud.

  double min_dist_to_foreground = std::numeric_limits<double>::max ();
  double closest_foreground_point[2];
  closest_foreground_point[0] = closest_foreground_point[1] = 0; // initial closest point is the first point?

  double initial_point[] = {0.0, 0.0};
  initial_point[0] = input_->points[point].x;
  initial_point[1] = input_->points[point].y;

  // Finding the closest foreground point to it
  for (size_t i_point = 0; i_point < foreground_points_.size (); i_point++)
  {
    double dist = 0.0;
    dist += (foreground_points_[i_point].x - initial_point[0]) * (foreground_points_[i_point].x - initial_point[0]);
    dist += (foreground_points_[i_point].y - initial_point[1]) * (foreground_points_[i_point].y - initial_point[1]);

    if (min_dist_to_foreground > dist)
    {
      min_dist_to_foreground = dist;
      closest_foreground_point[0] = foreground_points_[i_point].x;
      closest_foreground_point[1] = foreground_points_[i_point].y;
    }
  }


  // Apply background penalty
  sink_weight = pow (min_dist_to_foreground / radius_, 0.5);

  // Apply forground penalty
  source_weight = source_weight_;
  return;
/*
  if (background_points_.size () == 0)
    return;

  for (int i_point = 0; i_point < background_points_.size (); i_point++)
  {
    double dist = 0.0;
    dist += (background_points_[i_point].x - initial_point[0]) * (background_points_[i_point].x - initial_point[0]);
    dist += (background_points_[i_point].y - initial_point[1]) * (background_points_[i_point].y - initial_point[1]);
    if (min_dist_to_background > dist)
    {
      min_dist_to_background = dist;
      closest_background_point[0] = background_points_[i_point].x;
      closest_background_point[1] = background_points_[i_point].y;
    }
  }

  if (min_dist_to_background <= epsilon_)
  {
    source_weight = 0.0;
    sink_weight = 1.0;
    return;
  }

  source_weight = 1.0 / (1.0 + pow (min_dist_to_background / min_dist_to_foreground, 0.5));
  sink_weight = 1 - source_weight;
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // takes cloud indices
 bool
MinCut::addEdge (int source, int target, double weight)
{
  std::set<int>::iterator iter_out = edge_marker_[source].find (target);
  if ( iter_out != edge_marker_[source].end () )
    return (false);

  EdgeDescriptor edge;
  EdgeDescriptor reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( vertices_[source], vertices_[target], *graph_ );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( vertices_[target], vertices_[source], *graph_ );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);

  (*capacity_)[edge] = weight;
  (*capacity_)[reverse_edge] = 0.0;
  (*reverse_edges_)[edge] = reverse_edge;
  (*reverse_edges_)[reverse_edge] = edge;
  edge_marker_[source].insert (target);

  return (true);
}

 inline float euclidianDist(pcl::FPFHSignature33 &a, pcl::FPFHSignature33 &b){


     //printf("[");

     float sum = 0;
     for(int i = 0; i < 33; i++){
         sum += powf(a.histogram[i] - b.histogram[i], 2);
         //printf("(%f, %f) ", a.histogram[i], b.histogram[i]);
     }
     //printf("]\n");

     //printf("sum: %f, sqrt sum: %f\n", sum, sqrt(sum));

     return sqrt(sum);
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::calculateBinaryPotential (int source, int target) const
{
  double weight = 0.0;
  double distance = 0.0;
  distance += (input_->points[source].x - input_->points[target].x) * (input_->points[source].x - input_->points[target].x);
  distance += (input_->points[source].y - input_->points[target].y) * (input_->points[source].y - input_->points[target].y);
  distance += (input_->points[source].z - input_->points[target].z) * (input_->points[source].z - input_->points[target].z);
  distance *= inverse_sigma_;

  // Feature space distace
  float feature_dist = euclidianDist(fpfhs_->at(source), fpfhs_->at(target));

  weight = exp (-distance*feature_dist);

  //qDebug("Dist( %f ), FeatureDist( %f ), weight( %f )", distance, feature_dist, weight);

  return (weight);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::recalculateUnaryPotentials ()
{

  OutEdgeIterator src_edge_iter;
  OutEdgeIterator src_edge_end;
  std::pair<EdgeDescriptor, bool> sink_edge;

  // Itterate though all the edges
  // Set weights to the sink. Set weights on other edges
  // Seems like every node is connected to the source
  for (boost::tie (src_edge_iter, src_edge_end) = boost::out_edges (source_, *graph_); src_edge_iter != src_edge_end; src_edge_iter++)
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    sink_edge.second = false;
    calculateUnaryPotential (static_cast<int> (boost::target (*src_edge_iter, *graph_)), source_weight, sink_weight);
    // Lookup the edge from the current edge target to the sink
    sink_edge = boost::lookup_edge (boost::target (*src_edge_iter, *graph_), sink_, *graph_);
    // does this edge does not exist?
    if (!sink_edge.second)
      return (false);

    // Set edge weights
    (*capacity_)[*src_edge_iter] = source_weight; // source to target weight
    (*capacity_)[sink_edge.first] = sink_weight; // target to sink weight
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::recalculateBinaryPotentials ()
{

  int number_of_points = static_cast<int> (indices_->size ());

  VertexIterator vertex_iter;
  VertexIterator vertex_end;
  OutEdgeIterator edge_iter;
  OutEdgeIterator edge_end;

  std::vector< std::set<VertexDescriptor> > edge_marker;
  std::set<VertexDescriptor> out_edges_marker;
  edge_marker.clear ();
  edge_marker.resize (number_of_points + 2, out_edges_marker);

  for (boost::tie (vertex_iter, vertex_end) = boost::vertices (*graph_); vertex_iter != vertex_end; vertex_iter++)
  {
    VertexDescriptor source_vertex = *vertex_iter;
    if (source_vertex == source_ || source_vertex == sink_)
      continue;
    for (boost::tie (edge_iter, edge_end) = boost::out_edges (source_vertex, *graph_); edge_iter != edge_end; edge_iter++)
    {
      //If this is not the edge of the graph, but the reverse fictitious edge that is needed for the algorithm then continue
      EdgeDescriptor reverse_edge = (*reverse_edges_)[*edge_iter];
      if ((*capacity_)[reverse_edge] != 0.0)
        continue;

      //If we already changed weight for this edge then continue
      VertexDescriptor target_vertex = boost::target (*edge_iter, *graph_);
      std::set<VertexDescriptor>::iterator iter_out = edge_marker[static_cast<int> (source_vertex)].find (target_vertex);
      if ( iter_out != edge_marker[static_cast<int> (source_vertex)].end () )
        continue;

      if (target_vertex != source_ && target_vertex != sink_)
      {
        //Change weight and remember that this edges were updated
        double weight = calculateBinaryPotential (static_cast<int> (target_vertex), static_cast<int> (source_vertex));
        (*capacity_)[*edge_iter] = weight;
        edge_marker[static_cast<int> (source_vertex)].insert (target_vertex);
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::assembleLabels (ResidualCapacityMap& residual_capacity)
{
  std::vector<int> labels;
  labels.resize (input_->points.size (), 0);
  int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; i_point++)
    labels[(*indices_)[i_point]] = 1; // assined 1 if in index

  clusters_.clear ();

  pcl::PointIndices segment;
  clusters_.resize (2, segment);

  OutEdgeIterator edge_iter, edge_end;
  for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
  {
    if (labels[edge_iter->m_target] == 1)
    {
      if (residual_capacity[*edge_iter] > epsilon_)
        clusters_[1].indices.push_back (static_cast<int> (edge_iter->m_target));
      else
        clusters_[0].indices.push_back (static_cast<int> (edge_iter->m_target));
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MinCut::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    int num_of_pts_in_first_cluster = static_cast<int> (clusters_[0].indices.size ());
    int num_of_pts_in_second_cluster = static_cast<int> (clusters_[1].indices.size ());
    int number_of_points = num_of_pts_in_first_cluster + num_of_pts_in_second_cluster;
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
    unsigned char foreground_color[3] = {255, 255, 255};
    unsigned char background_color[3] = {255, 0, 0};
    colored_cloud->width = number_of_points;
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_->is_dense;

    pcl::PointXYZRGB point;
    int point_index = 0;
    for (int i_point = 0; i_point < num_of_pts_in_first_cluster; i_point++)
    {
      point_index = clusters_[0].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = background_color[0];
      point.g = background_color[1];
      point.b = background_color[2];
      colored_cloud->points.push_back (point);
    }

    for (int i_point = 0; i_point < num_of_pts_in_second_cluster; i_point++)
    {
      point_index = clusters_[1].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = foreground_color[0];
      point.g = foreground_color[1];
      point.b = foreground_color[2];
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

 bool
 MinCut::deinitCompute ()
 {
   // Reset the indices
  if (fake_indices_)
   {
     indices_.reset ();
     fake_indices_ = false;
   }
   return (true);
 }

 bool
MinCut::initCompute ()
 {
   // Check if input was set
   if (!input_)
     return (false);

   // If no point indices have been given, construct a set of indices for the entire input point cloud
   if (!indices_)
   {
     fake_indices_ = true;
     std::vector<int> *indices = new std::vector<int> (input_->width * input_->height);
     for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }
     indices_.reset (indices);
   }
   return (true);
 }

 /*MinCut::setHorisontalRadius(bool val){
     horisonal_radius_ = val;
 }*/