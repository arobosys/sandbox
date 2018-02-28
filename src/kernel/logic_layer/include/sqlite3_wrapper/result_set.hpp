//
// Created by vladislav on 15.09.16.
//

#ifndef PROJECT_RESULTSET_HPP
#define PROJECT_RESULTSET_HPP

#include <unordered_map>
#include <vector>

class ResultSetReadError : public std::runtime_error {
public:
    ResultSetReadError(const std::string &addInfo = "")
            : std::runtime_error("Database error. " + addInfo) {}
};

/**
 * @brief Представляет результат, полученный из базы данных
 */
class ResultSet {
public:
    /**
     * @brief Представляет кортеж, вставляемый в базу. Выглядит как массив пар "Имя колонки - Значение в кортеже"
     */
    typedef std::vector<std::pair<std::string, std::string>> row_type;

    /**
     * @brief Представляет таблицу результата. Выглядит как "Имя столбца - Массив значений столбца"
     */
    typedef std::unordered_map<std::string, std::vector<std::string>> result_type;

private:
    mutable result_type result;
    mutable int currentRow = -1;
    int rowsCount = 0;

public:
    void addRow(const row_type& row);

    std::string getString(const std::string& columnName) const;
    int getInteger(const std::string& columnName) const;
    double getReal(const std::string& columnName) const;
    bool getBoolean(const std::string& columnName) const;
    bool isNull(const std::string& columnName) const;

    int getRowsCount() const;
    int getColumnsCount() const;

    bool next() const;
    void clear();
};


#endif //PROJECT_RESULTSET_HPP
