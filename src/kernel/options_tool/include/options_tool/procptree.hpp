#ifndef PROC_PTREE_HPP_
#define PROC_PTREE_HPP_

#include <string>
#include <vector>
#include <stdexcept>
#include <boost/optional.hpp>

/**
 * @brief Exception class. Thrown if given path doesn't exist in property tree
 */
struct ProcPropTreeBadPath : public std::runtime_error {
    ProcPropTreeBadPath(const std::string & addInfo = "") 
        : std::runtime_error("Process property tree bad path. " + addInfo)
    {}
};

struct ProcPropTree;

#include <memory>
typedef std::shared_ptr<ProcPropTree> ProcPropTreePtr;

struct ProcPropTree {
    typedef const std::string & PathTp;

    /**
     * @brief get integer value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual int getInt(PathTp path = "") const = 0;

    /**
     * @brief get string value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::string getString(PathTp path = "") const = 0;

    /**
     * @brief get double value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual double getDouble(PathTp path = "") const = 0;

    /**
     * @brief get boolean value from specified path
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual bool getBool(PathTp path = "") const = 0;

    /**
     * @brief get int value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<int> getIntOpt(PathTp path = "") const = 0;

    /**
     * @brief get string value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::string> getStringOpt(PathTp path = "") const = 0;
    
    /**
     * @brief get double value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<double> getDoubleOpt(PathTp path = "") const = 0;

    /**
     * @brief get boolean value from specified path (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<bool> getBoolOpt(PathTp path = "") const = 0;

    /**
     * @brief get subtree with root node = given path
     */
    virtual ProcPropTreePtr getSubtree(PathTp path = "") const = 0;

    /**
     * @brief get subtree with root node = given path (in optional wrapper)
     */
    virtual boost::optional<ProcPropTreePtr> getSubtreeOptional(PathTp path = "") const = 0;
    
    /**
     * @brief get int array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<int> getArrayInt(PathTp path = "") const = 0;
    
    /**
     * @brief get string array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
     */
    virtual std::vector<std::string> getArrayString(PathTp path = "") const = 0;
    
    /**
     * @brief get double array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<double> getArrayDouble(PathTp path = "") const = 0;
    
    /**
     * @brief get bool array
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<bool> getArrayBool(PathTp path = "") const = 0;
    
    /**
     * @brief get array of subtrees
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType
     */
    virtual std::vector<ProcPropTreePtr> getArraySubtree(PathTp path = "") const = 0;
    
    /**
     * @brief get int array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<int>> getArrayIntOptional(PathTp path = "") const = 0;
    
    /**
     * @brief get string array  (in optional wrapper).
     *        Can throw ProcPropTreeBadPath, ProcPropTreeBadType  
     */
    virtual boost::optional<std::vector<std::string>> getArrayStringOptional(PathTp path = "") const = 0;
    
    /**
     * @brief get double array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<double>> getArrayDoubleOptional(PathTp path = "") const = 0;
    
    /**
     * @brief get bool array  (in optional wrapper).
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<bool>> getArrayBoolOptional(PathTp path = "") const = 0;
    
    /**
     * @brief get array of subtrees (in optional wrapper)
     *        Can throw ProcPropTreeBadType
     */
    virtual boost::optional<std::vector<ProcPropTreePtr>> getArraySubtreeOptional(PathTp path = "") const = 0;
    
    virtual void putInt(PathTp path, int val) = 0;
    virtual void putString(PathTp path, const std::string & val) = 0;
    virtual void putDouble(PathTp path, double val) = 0;
    virtual void putBool(PathTp path, bool val) = 0;
    virtual void putSubtree(PathTp path, const ProcPropTreePtr val) = 0;
    virtual void putArrayInt(PathTp path, const std::vector<int> & vals) = 0;
    virtual void putArrayString(PathTp path, const std::vector<std::string> & vals) = 0;
    virtual void putArrayDouble(PathTp path, const std::vector<double> & vals) = 0;
    virtual void putArraySubtree(PathTp path, const std::vector<ProcPropTreePtr> & vals) = 0;

    /**
     * @brief transform tree into JSON string
     */
    virtual std::string JSONize() const = 0;

    virtual ~ProcPropTree() {}
};


#endif // PROC_PTREE_HPP_
