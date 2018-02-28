//
// Created by vladislav on 30.06.16.
//

#include "sqlite3_wrapper/database.hpp"
#include <fstream>

Database::Database(const std::string& filename, bool fileMustExist) {
    if (fileMustExist && !isFileExists(filename)) {
        throw DatabaseRuntimeError("Can't connect to database. File not exists.");
    }

    if ((sqlite3_open(filename.c_str(), &db)) != SQLITE_OK) {
        sqlite3_close(db);
        throw DatabaseRuntimeError("Can't connect to database: " + getErrorMessage());
    }
}

Database::~Database() {
    sqlite3_close(db);
}

bool Database::isFileExists(const std::string& fileName) {
    std::ifstream infile(fileName);
    return infile.good();
}

std::string Database::getErrorMessage() {
    return sqlite3_errmsg(db);
}

sqlite3_stmt* Database::prepare(const std::string& query) {
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, NULL) != SQLITE_OK)
        throw DatabaseRuntimeError("Can't prepare SQL Query. Message: " + getErrorMessage());

    return stmt;
}

int Database::set(sqlite3_stmt* stmt, int pos, const std::string& value) {
    return sqlite3_bind_text(stmt, pos, value.c_str(), -1, SQLITE_STATIC);
}

int Database::set(sqlite3_stmt* stmt, int pos, int value) {
    return sqlite3_bind_int(stmt, pos, value);
}

int Database::set(sqlite3_stmt* stmt, int pos, double value)  {
    return sqlite3_bind_double(stmt, pos, value);
}

int Database::set(sqlite3_stmt* stmt, int pos, std::nullptr_t) {
    return sqlite3_bind_null(stmt, pos);
}

void Database::execQuery(sqlite3_stmt* stmt, const handlerFunc& handler) {
    int rc;
    int columns = sqlite3_column_count(stmt);
    ResultSet resultSet;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        ResultSet::row_type row;
        for (int i = 0; i < columns; ++i) {
            std::string column_name = sqlite3_column_name(stmt, i);
            std::string value;
            switch (sqlite3_column_type(stmt, i)) {
                case SQLITE_INTEGER:
                    value = std::to_string(sqlite3_column_int(stmt, i));
                    break;
                case SQLITE_FLOAT:
                    value = std::to_string(sqlite3_column_double(stmt, i));
                    break;
                case SQLITE_TEXT:
                    value = reinterpret_cast<const char*>(sqlite3_column_text(stmt, i));
                    break;
                case SQLITE_BLOB:
                    value = reinterpret_cast<const char*>(sqlite3_column_blob(stmt, i));
                    break;
                case SQLITE_NULL:
                    value = "{null}";
                    break;
            }
            row.emplace_back(column_name, value);
        }
        resultSet.addRow(row);
    }

    if (rc != SQLITE_DONE) {
        throw DatabaseRuntimeError("Can't step SQL Query. Message: " + getErrorMessage());
    }

    rc = sqlite3_finalize(stmt);

    if (rc != SQLITE_OK) {
        throw DatabaseRuntimeError("Can't finalize SQL Query. Message: " + getErrorMessage());
    }

    if (handler)
        return handler(resultSet);
}