#ifndef PROCPTREE_HANDMADE_HPP_
#define PROCPTREE_HANDMADE_HPP_

#include "procptree.hpp"
#include <boost/variant.hpp>
#include <unordered_map>


struct ProcPropTreeHandmade {
private:
    std::vector<boost::variant<int, std::string, double, bool, ProcPropTreePtr>> childs;
    std::unordered_map<std::string, size_t> nameToIdx;
    
    struct PathSpec {
        std::string propName;
        size_t propIdx;
        PathSpec(const std::string & propName, size_t propIdx = 0) : propName(propName), propIdx(propIdx) {}
    };
    
    std::pair<PathSpec, std::string> pathHeadTail(const std::string & path) {
    }
public:
    typedef const std::string & PathTp;

    /**
     * @brief get integer value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual int getInt(PathTp path = "") const {}

    /**
     * @brief get string value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::string getString(PathTp path = "") const {}

    /**
     * @brief get double value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual double getDouble(PathTp path = "") const {}

    /**
     * @brief get boolean value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual bool getBool(PathTp path = "") const {}

    /**
     * @brief get int value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<int> getIntOpt(PathTp path = "") const {}

    /**
     * @brief get string value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::string> getStringOpt(PathTp path = "") const {}
    
    /**
     * @brief get double value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<double> getDoubleOpt(PathTp path = "") const {}

    /**
     * @brief get boolean value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<bool> getBoolOpt(PathTp path = "") const {}

    /**
     * @brief get subtree with root node = given path
     */
    virtual ProcPropTreePtr getSubtree(PathTp path = "") const {}

    /**
     * @brief get subtree with root node = given path (in optional wrapper)
     */
    virtual boost::optional<ProcPropTreePtr> getSubtreeOptional(PathTp path = "") const {}
    
    /**
     * @brief get int array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<int> getArrayInt(PathTp path = "") const {}
    
    /**
     * @brief get string array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
     */
    virtual std::vector<std::string> getArrayString(PathTp path = "") const {}
    
    /**
     * @brief get double array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<double> getArrayDouble(PathTp path = "") const {}
    
    /**
     * @brief get bool array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<bool> getArrayBool(PathTp path = "") const {}
    
    /**
     * @brief get array of subtrees
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<ProcPropTreePtr> getArraySubtree(PathTp path = "") const {}
    
    /**
     * @brief get int array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<int>> getArrayIntOptional(PathTp path = "") const {}
    
    /**
     * @brief get string array  (in optional wrapper).
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
     */
    virtual boost::optional<std::vector<std::string>> getArrayStringOptional(PathTp path = "") const {}
    
    /**
     * @brief get double array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<double>> getArrayDoubleOptional(PathTp path = "") const {}
    
    /**
     * @brief get bool array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<bool>> getArrayBoolOptional(PathTp path = "") const {}
    
    /**
     * @brief get array of subtrees (in optional wrapper)
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<ProcPropTreePtr>> getArraySubtreeOptional(PathTp path = "") const {}
    
    virtual void putInt(PathTp path, int val) {}
    virtual void putString(PathTp path, const std::string & val) {}
    virtual void putDouble(PathTp path, double val) {}
    virtual void putBool(PathTp path, bool val) {}
    virtual void putSubtree(PathTp path, const ProcPropTreePtr val) {}
    virtual void putArrayInt(PathTp path, const std::vector<int> & vals) {}
    virtual void putArrayString(PathTp path, const std::vector<std::string> & vals) {}
    virtual void putArrayDouble(PathTp path, const std::vector<double> & vals) {}
    virtual void putArraySubtree(PathTp path, const std::vector<ProcPropTreePtr> & vals) {}

    /**
     * @brief transform tree into JSON string
     */
    virtual std::string JSONize() const {}

    virtual ~ProcPropTree() {}
};

#endif