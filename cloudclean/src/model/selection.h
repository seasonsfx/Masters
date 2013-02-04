#ifndef MODEL_CSELECTION_H_
#define MODEL_CSELECTION_H_

#include <vector>
#include <map>

/**
 * @brief The Selection class
 * Lists all the points selected in a class. This should map a cpointcloud to
 * a list of indexes.
 */
class Selection {
 public:
    Selection();
 public:
    std::map<int, std::vector<unsigned int> > selection_list;
};

#endif  // MODEL_CSELECTION_H_
