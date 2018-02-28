#include "options_tool/procptree_boostptree.hpp"

class ProcPropTreeBoost;

template <typename T>
std::vector<T> boostptree_as_vector(boost::property_tree::ptree const& pt, boost::property_tree::ptree::key_type const& key)
{
    std::vector<T> r;
    for (auto& item : pt.get_child(key))
        r.push_back(item.second.get_value<T>());
    return r;
}
#include <iostream>
template <>
std::vector<ProcPropTreePtr> boostptree_as_vector(boost::property_tree::ptree const & pt, boost::property_tree::ptree::key_type const & key) {
    std::vector<ProcPropTreePtr> r;
    for (auto & item : pt.get_child(key)) {
        //std::cout << "boostptree_as_vector: " << ProcPropTreeBoost(item.second).JSONize() << std::endl; fflush(stdout);
        r.push_back(std::dynamic_pointer_cast<ProcPropTree>(std::make_shared<ProcPropTreeBoost>(item.second, true)));
    }
    return r;
}

template <typename T>
boost::optional<std::vector<T>> boostptree_as_vector_opt(boost::property_tree::ptree const& pt, boost::property_tree::ptree::key_type const& key)
{
    if (!pt.get_child_optional(key))
        return boost::optional<std::vector<T>>();
    return boostptree_as_vector<T>(pt, key);
}

ProcPropTreeBoost::ProcPropTreeBoost() 
    : ptobj(), tree(ptobj)
{
}

/**
 * @brief Ctor. Takes boosts' property tree and stores copu of it inside.
 */
ProcPropTreeBoost::ProcPropTreeBoost(const boost::property_tree::ptree & tree, bool mkcopy)
    : ptobj(), tree(ptobj = tree) 
{}

/**
 * @brief Ctor. From json string
 */
ProcPropTreeBoost::ProcPropTreeBoost(const std::string & jsonStr) : ptobj(), tree(ptobj) {
    std::stringstream ss;
    ss << jsonStr;
    boost::property_tree::read_json(ss, ptobj);
}

/**
 * @brief get integer value from specified path
 *        Can throw ProcPropTreeBadPath, 
 */
int ProcPropTreeBoost::getInt(PathTp path) const {
    return tree.get<int>(path);
}

/**
 * @brief get string value from specified path
 *        Can throw ProcPropTreeBadPath, 
 */
std::string ProcPropTreeBoost::getString(PathTp path) const {
    return tree.get<std::string>(path);
}

/**
 * @brief get double value from specified path
 *        Can throw ProcPropTreeBadPath, 
 */
double ProcPropTreeBoost::getDouble(PathTp path) const {
    return tree.get<double>(path);
}

/**
 * @brief get boolean value from specified path
 *        Can throw ProcPropTreeBadPath, 
 */
bool ProcPropTreeBoost::getBool(PathTp path) const {
    return tree.get<bool>(path);
}

/**
 * @brief get int value from specified path (in boost::optional wrapper).
 */
boost::optional<int> ProcPropTreeBoost::getIntOpt(PathTp path) const {
    return tree.get_optional<int>(path);
}

/**
 * @brief get string value from specified path (in boost::optional wrapper).
 */
boost::optional<std::string> ProcPropTreeBoost::getStringOpt(PathTp path) const {
    return tree.get_optional<std::string>(path);
}

/**
 * @brief get double value from specified path (in boost::optional wrapper).
 */
boost::optional<double> ProcPropTreeBoost::getDoubleOpt(PathTp path) const {
    return tree.get_optional<double>(path);
}

/**
 * @brief get boolean value from specified path (in boost::optional wrapper).
 */
boost::optional<bool> ProcPropTreeBoost::getBoolOpt(PathTp path) const {
    return tree.get_optional<bool>(path);
}

/**
 * @brief get subtree with root = given path
 */
ProcPropTreePtr ProcPropTreeBoost::getSubtree(PathTp path) const {
    return std::dynamic_pointer_cast<ProcPropTree>(std::make_shared<ProcPropTreeBoost>(tree.get_child(path), false));
}

/**
 * @brief get subtree with root node = given path (in optional wrapper)
 */
boost::optional<ProcPropTreePtr> ProcPropTreeBoost::getSubtreeOptional(PathTp path) const {
    auto opt_boost_ptree = tree.get_child_optional(path);
    if (opt_boost_ptree.is_initialized()) {
        return boost::optional<ProcPropTreePtr>(getSubtree(path));
    } else
        return boost::optional<ProcPropTreePtr>();
}

/**
 * @brief get int array
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
 */
std::vector<int> ProcPropTreeBoost::getArrayInt(PathTp path) const {
    return boostptree_as_vector<int>(tree, path);
}

/**
 * @brief get string array
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
 */
std::vector<std::string> ProcPropTreeBoost::getArrayString(PathTp path) const {
    return boostptree_as_vector<std::string>(tree, path);
}

/**
 * @brief get double array
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
 */
std::vector<double> ProcPropTreeBoost::getArrayDouble(PathTp path) const {
    return boostptree_as_vector<double>(tree, path);
}

/**
 * @brief get bool array
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
 */
std::vector<bool> ProcPropTreeBoost::getArrayBool(PathTp path) const {
    return boostptree_as_vector<bool>(tree, path);
}

#include <iostream>
/**
 * @brief get array of subtrees
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
 */
std::vector<ProcPropTreePtr> ProcPropTreeBoost::getArraySubtree(PathTp path) const {
    //std::cout << "getArraySubtree:" << JSONize() << std::endl; fflush(stdout);
    return boostptree_as_vector<ProcPropTreePtr>(tree, path);
}

/**
 * @brief get int array  (in optional wrapper).
 *        Can throw ProcPropTreeBadType
 */
boost::optional<std::vector<int>> ProcPropTreeBoost::getArrayIntOptional(PathTp path) const {
    return boostptree_as_vector_opt<int>(tree, path);
}

/**
 * @brief get string array  (in optional wrapper).
 *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
 */
boost::optional<std::vector<std::string>> ProcPropTreeBoost::getArrayStringOptional(PathTp path) const {
    return boostptree_as_vector_opt<std::string>(tree, path);
}

/**
 * @brief get double array  (in optional wrapper).
 *        Can throw ProcPropTreeBadType
 */
boost::optional<std::vector<double>> ProcPropTreeBoost::getArrayDoubleOptional(PathTp path) const {
    return boostptree_as_vector_opt<double>(tree, path);
}

/**
 * @brief get bool array  (in optional wrapper).
 *        Can throw ProcPropTreeBadType
 */
boost::optional<std::vector<bool>> ProcPropTreeBoost::getArrayBoolOptional(PathTp path) const {
    return boostptree_as_vector_opt<bool>(tree, path);
}   

/**
 * @brief get array of subtrees (in optional wrapper)
 *        Can throw ProcPropTreeBadType
 */
boost::optional<std::vector<ProcPropTreePtr>> ProcPropTreeBoost::getArraySubtreeOptional(PathTp path) const {
    return boostptree_as_vector_opt<ProcPropTreePtr>(tree, path);
}


/**
 * @brief transform tree into JSON string
 */
std::string ProcPropTreeBoost::JSONize() const {
    std::stringstream ss;
    boost::property_tree::write_json(ss, tree);
    return ss.str();
}

void ProcPropTreeBoost::putInt(PathTp path, int val) {
    tree.put(path, val);
}

void ProcPropTreeBoost::putString(PathTp path, const std::string & val) {
    tree.put(path, val);
}

void ProcPropTreeBoost::putDouble(PathTp path, double val) {
    tree.put(path, val);
}

void ProcPropTreeBoost::putBool(PathTp path, bool val) {
    tree.put(path, val);
}

void ProcPropTreeBoost::putSubtree(PathTp path, const ProcPropTreePtr val) {
    // HACK
    tree.put_child(path, std::static_pointer_cast<ProcPropTreeBoost>(val)->tree);
}

template <typename ArrTp>
void putArrayPT(boost::property_tree::ptree & tree, ProcPropTree::PathTp path, const ArrTp & vals) {
    boost::property_tree::ptree child;
    for (auto & val : vals) {
        boost::property_tree::ptree ptreeVal;
        ptreeVal.put("", val);
        child.push_back(std::make_pair("", std::move(ptreeVal)));
    }
    tree.put_child(path, std::move(child));
}

template<>
void putArrayPT<std::vector<ProcPropTreePtr>>(boost::property_tree::ptree & tree, ProcPropTree::PathTp path, const std::vector<ProcPropTreePtr> & vals) {
    boost::property_tree::ptree child;
    for (ProcPropTreePtr val : vals) {
        // HACK
        child.push_back(std::make_pair("", std::static_pointer_cast<ProcPropTreeBoost>(val)->tree));
    }
    tree.put_child(path, std::move(child));
}

void ProcPropTreeBoost::putArrayInt(PathTp path, const std::vector<int> & vals) {
    putArrayPT(tree, path, vals);
}

void ProcPropTreeBoost::putArrayString(PathTp path, const std::vector<std::string> & vals) {
    putArrayPT(tree, path, vals);
}

void ProcPropTreeBoost::putArrayDouble(PathTp path, const std::vector<double> & vals) {
    putArrayPT(tree, path, vals);
}

void ProcPropTreeBoost::putArraySubtree(PathTp path, const std::vector<ProcPropTreePtr> & vals) {
    putArrayPT(tree, path, vals);
}

ProcPropTreeBoost::~ProcPropTreeBoost() {} 
