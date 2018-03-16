//
// Created by vladislav on 23.09.16.
//

#include <cstdlib>
#include <stdexcept>

/***
 * Get AVIS_ROOT environment variable
 * @return AVIS_ROOT environment variable
 * @throws std::runtime_error if variable is not found
 */
std::string getAvisRoot() {
    char *path = std::getenv("AVIS_ROOT");
    if (path == nullptr) {
        throw std::runtime_error("Try to set up AVIS_ROOT in ~/.profile");
    }
    return path;
}