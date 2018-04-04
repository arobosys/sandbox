//
// Created by vladislav on 21.10.16.
//

#ifndef PROJECT_STATEGROUP_HPP
#define PROJECT_STATEGROUP_HPP

#include <string>
#include <vector>

/**
 * @brief Структура данных, описывающая возможный результат, принятый с процесса
 * @desc Содержит массив пар ключ-значение, которые однозначно определяют state или запрос к пользователю.
 *       Если с процесса приходит несколько таких групп, выбирается наиболее приоритетная
 */
struct ProcessGroup {
    typedef std::pair<std::string, std::string> keyValue_t;
    typedef std::vector<keyValue_t> group_t;

    int id;
    group_t group;
    bool isRequest;
    int stateId;
    int requestId;
    int priority;

    /**
     * @brief Проверяет, есть ли в group группа, содержащаяся в этом экземпляре
     */
    bool match(const group_t& group) const;
};


#endif //PROJECT_STATEGROUP_HPP
