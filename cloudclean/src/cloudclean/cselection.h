#ifndef CSELECTION_H_
#define CSELECTION_H_

#include <vector>
#include <map>

/**
 * @brief The CSelection class
 * Lists all the points selected in a class. This should map a cpointcloud to
 * a list of indexes.
 */
class CSelection
{
 public:
    CSelection();
 public:
    std::map<int, std::vector<unsigned int> > selection_list;
};

#endif  // CSELECTION_H_
