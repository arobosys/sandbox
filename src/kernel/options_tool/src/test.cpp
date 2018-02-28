#include <cstdlib>
#include <fstream>
#include <iostream>
#include "options_tool/optionstooljson.hpp"

int main(int argc, char **argv) {
    std::string options = "{ \"mode\": \"manual\", \"delta\": 1, \"p_coff\": 2 }";
    OptionsToolJSON tool("/home/v/schema.json", true);
    std::cout << "isSyntaxCorrect: " << tool.isSyntaxCorrect(options) << std::endl;
    auto tree = tool.parseOptionsString(options);
    std::cout << "mode: " << tree->getString("mode") << std::endl;
    std::cout << "delta: " << tree->getInt("delta") << std::endl;
    std::cout << "p_coff: " << tree->getInt("p_coff") << std::endl;
    return EXIT_SUCCESS;
}
