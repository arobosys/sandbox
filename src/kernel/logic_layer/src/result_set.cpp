//
// Created by vladislav on 15.09.16.
//

#include "sqlite3_wrapper/result_set.hpp"

void ResultSet::addRow(const row_type& row) {
    for (const auto& r : row)
        result[r.first].push_back(r.second);
    ++rowsCount;
}

std::string ResultSet::getString(const std::string &columnName) const {
    if (result.find(columnName) == result.end() || currentRow >= rowsCount)
        throw ResultSetReadError("Can't read from result set");

    return result[columnName][currentRow];
}

int ResultSet::getInteger(const std::string &columnName) const {
    return std::stoi(getString(columnName));
}

double ResultSet::getReal(const std::string &columnName) const {
    return std::stod(getString(columnName));
}

bool ResultSet::getBoolean(const std::string &columnName) const {
    return std::stoi(getString(columnName)) != 0;
}

bool ResultSet::isNull(const std::string& columnName) const {
    return getString(columnName) == "{null}";
}

bool ResultSet::next() const {
    bool n;
    if ((n = currentRow < rowsCount - 1))
        ++currentRow;
    return n;
}

void ResultSet::clear() {
    currentRow = -1;
    rowsCount = 0;
    result.clear();
}

int ResultSet::getRowsCount() const {
    return rowsCount;
}

int ResultSet::getColumnsCount() const {
    return (int)result.size();
}