//
// Created by vladislav on 24.09.16.
//

#ifndef PROJECT_MAPPING_HPP
#define PROJECT_MAPPING_HPP

#include "sqlite3_wrapper/database.hpp"
#include "scenario_executor.hpp"
#include "scenario_node.hpp"
#include "techmap_node.hpp"
#include "process_group.hpp"
#include "process_request.hpp"
#include <core_msgs/ProcessData.h>

/**
 * @desc ORM класс для Logic Layer (Object-Relational Mapping)
 */
class Mapping {
    static const char* HLLScenarioQuery;
    static const char* HLLEdgesQuery;
    static const char* HLLNodesQuery;
    static const char* HLLParamsQuery;

    static const char* LLLStartIdQuery;
    static const char* LLLEdgesQuery;
    static const char* LLLNodesQuery;
    static const char* LLLTLQuery;
    static const char* LLLParamsQuery;

    static const char* ChildProcessesParamsQuery;
    static const char* ChildProcessesQuery;
    static const char* ProcessGroupQuery;
    static const char* StateQuery;
    static const char* RequestQuery;

    static DatabasePtr DATABASE;

    typedef std::vector<core_msgs::ProcessData> ProcessVec;
    typedef ScenarioExecutor::params_type params_type;

    /**
     * @brief Собирает информацию по заданному Task List
     * @param id Номер TL
     * @param pData Массив, в который записываются данные о процессах данного TL
     */
    static void get_task_list_by_id(int id, ProcessVec& pData);

    /**
     * @brief Перемещает параметры из resultSet в paramsVec
     * @param paramsVec Массив параметров
     * @param resultSet Результат запроса из БД
     */
    static void get_params(params_type& paramsVec, const ResultSet& resultSet);

    /**
     * @brief Получает параметры для дочернихп процессов из resultSet в paramsVec
     * @param paramsVec Массив параметров
     * @param resultSet Результат запроса из БД
     * @param idTL Номер TaskList корневого процесса
     * @param prePath Путь из ID до корневого процесса
     */
    static void get_child_params(params_type& paramsVec, const ResultSet& resultSet, const std::string& prePath);

    /**
     * @brief Превращает путь, разделенный '/', в массив int
     * @param path Путь в строковом формате
     * @return Путь в формате массива int
     */
    static std::vector<int> parse_path(const std::string& path);

public:

    /**
     * @brief Устанавливает экземпляр базы данных
     * @param database Экземпляр Database
     */
    static void setDatabase(const DatabasePtr& database);

    /**
     * @brief Конструирует сценарий HLL по заданному id
     * @param id Номер сценария
     * @return Сценарий HLL
     */
    static ScenarioExecutorPtr get_scenario_by_name(const std::string& alias);

    /**
     * @brief Конструирует техкарту LLL по заданному id
     * @param id Номер сценария
     * @return Техкарта LLL
     */
    static ScenarioExecutorPtr get_tech_map_by_id(int id);

    /**
     * @brief Возвращает информацию о всех дочерних процессах для данного процесса
     * @param id Номер корневого процессса
     * @return Массив с данными о дочерних процессах
     */
    static std::vector<core_msgs::ChildProcess> get_child_processes_by_id(int id);

    /**
     * @brief Возращает всевозможные группы результатов процесса
     * @param id Номер процесса
     * @return Массив групп результатов процесса
     */
    static std::vector<ProcessGroup> get_process_group(int id);

    /**
     * @brief Возвращает state по номеру
     * @param id Номер state
     * @return State в формате строки
     */
    static std::string get_state(int id);

    /**
     * @brief Возвращает запрос к пользователю по номеру
     * @param id Номер запроса
     * @return Запрос к пользователю
     */
    static ProcessRequest get_request(int id);
};

#endif //PROJECT_MAPPING_HPP
