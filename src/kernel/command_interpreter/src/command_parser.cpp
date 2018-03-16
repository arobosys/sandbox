#include "command_parser.hpp"
#include <sstream>
#include <unordered_map>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <regex>
#include <ros/ros.h>
#include <core_msgs/CI_to_HLLAction.h>

const std::string command_parser::usage = std::string("Usage: \"start alias ID_TL ID_PROC [key value] [key value] ...")+
										  std::string(" ID_TL ID_PROC [key value] [key value] ...\"");

const std::string command_parser::START = "start";
const std::string command_parser::STOP = "stop";
const std::string command_parser::RESTART = "restart";

bool isID(std::string &str){
	auto it = str.begin();
	while(it != str.end() && std::isdigit(*it)) ++it;
	return !str.empty() && it == str.end();
}
bool isValidCommand(std::string &str){
	return boost::iequals(command_parser::START, str)
		   || boost::iequals(command_parser::STOP, str)
		   || boost::iequals(command_parser::RESTART, str);
}

core_msgs::CI_to_HLLGoal command_parser::parse(const std::string &command) {
	core_msgs::CI_to_HLLGoal res;

	std::stringstream ss(command);
	std::string cur = "";
	std::vector<std::string> tokens;
	while (std::getline(ss, cur, ' ')) {
		if (!cur.empty()) {
			tokens.push_back(cur);
		}
	}

	if (tokens.empty()) {
		throw ParseError("Empty input string");
	} else {
		res.command = tokens[0];
		if(!isValidCommand(res.command)){
			throw ParseError("Unknown command \""+tokens[0]+"\"");
		}
		int i = 1;

		if(!isID(tokens[1])){
			res.alias = tokens[1];
			i = 2;
		}
		if(tokens.size() % 2 != i % 2) throw ParseError("Wrong number of arguments");

		while(i < tokens.size()-1){
			core_msgs::LL_Params llparams;
			if(!isID(tokens[i]) || !isID(tokens[i+1])) throw ParseError("Unexpected args: \""+tokens[i]+"\"; \""+tokens[i+1]+"\"");
			llparams.ID_Proc = std::stoi(tokens[i+1]);
			llparams.ID_TL = std::stoi(tokens[i]);
			i+=2;
			if(i >= tokens.size()-1) break;
			while(!isID(tokens[i]) && !isID(tokens[i+1])){
				core_msgs::KeyValue kv;
				kv.key = tokens[i];
				kv.value = tokens[i+1];
				llparams.key_value.push_back(kv);
				i+=2;
				if(i >= tokens.size()-1) break;
			}
			res.params.push_back(llparams);
		}
	}
	return res;
}
