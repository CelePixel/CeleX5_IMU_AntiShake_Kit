/************************************************************

FileName: INIParser.h

Author: yueyin.zhou@celepixel.com	Date:2018/10/16

ini file parser 

Version: 0.1

Function List:


History:
ZHOU Yueyin    2018/10/16     0.1	build this module

ref: https://blog.csdn.net/normallife/article/details/52661632
***********************************************************/

#ifndef INI_PARSER_H
#define INI_PARSER_H

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(INIPARSER_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <map>

#include "logger.h"
#include "ErrCode.h"

#define INI_INFO LOG_INFO("INIParser")
#define INI_WARN LOG_WARN("INIParser")
#define INI_ERROR LOG_ERROR("INIParser")
#define INI_FATAL LOG_FATAL("INIParser")

namespace util{
class ININode
{
public:
	ININode(std::string root, std::string key, std::string value)
	{
		this->root_ = root;
		this->key_ = key;
		this->value_ = value;
	}
	std::string root_;
	std::string key_;
	std::string value_;
};

/*template class CELEX_EXPORTS std::map<std::string, std::string>;*/
class SubNode
{
public:
	void InsertElement(std::string key, std::string value)
	{
		sub_node.insert(std::pair<std::string, std::string>(key, value));
	}
	void Clear() { sub_node.clear(); }
	std::map<std::string, std::string> sub_node;
};


/*template class CELEX_EXPORTS std::map<std::string, SubNode>;*/
class CELEX_EXPORTS INIParser
{
public:
	
	INIParser();
	~INIParser();

	int ReadINI(std::string path);
	std::string GetValue(std::string root, std::string key);
	std::vector<ININode>::size_type GetSize() { return map_ini.size(); }
	std::vector<ININode>::size_type SetValue(std::string root, std::string key, std::string value);
	int WriteINI(std::string path);
	void Clear() { map_ini.clear(); }

private:
	std::map<std::string, SubNode> map_ini;
};		//class
}		//namespace
#endif	//INI_PARSER_H